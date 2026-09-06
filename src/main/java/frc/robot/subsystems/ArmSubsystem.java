// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private static final double MINIMUM_VALID_CALIBRATION_ROTATIONS = 1e-6;
  private static final double MINIMUM_VELOCITY_TIMESTEP_SECONDS = 1e-6;
  private static final double VELOCITY_FILTER_ALPHA = 0.3;
  private static final double HOLD_ANGLE_FILTER_ALPHA = 0.2;

  public record ArmEncoderPositions(double motor1Rotations, double motor2Rotations) {
    public double averageRotations() {
      return (motor1Rotations + motor2Rotations) / 2.0;
    }

    public double differenceRotations() {
      return Math.abs(motor1Rotations - motor2Rotations);
    }
  }

  public interface ArmIO {
    ArmEncoderPositions getEncoderPositions();

    void setVoltage(double voltage);

    void zeroEncoders();
  }

  private final ArmIO io;
  private final DoubleSupplier voltageSupplier;
  private final DoubleSupplier holdVoltageSupplier;
  private final DoubleSupplier maxSpeedDegreesPerSecondSupplier;
  private final DoubleSupplier voltageSlewVoltsPerSecondSupplier;
  private final double targetAngleDegrees;
  private final OptionalDouble motorRotationsAtMaxAngle;
  private ArmEncoderPositions encoderPositions;
  private double appliedVoltage;
  private double holdSetpointDegrees;
  private boolean holdNudgeActive;
  private double previousAngleDegrees;
  private double previousTimestampSeconds;
  // Package-private for tests: the filtered velocity is what the speed ceiling acts on.
  double angleVelocityDegreesPerSecond;
  private double filteredHoldAngleDegrees;
  private boolean reachedUpperLimit;
  // Latched when a movement command starts: the mid-angle command keeps its direction for
  // its whole run (the elevator's mid command uses the same pattern).
  private boolean midMovementIsUpward;
  private double lastMovementVoltage;
  private double lastMovementTimestampSeconds;
  private double moveStartTimestampSeconds;
  private double lastMoveDurationSeconds = -1.0;
  private String lastMoveActionName = "";

  public ArmSubsystem(
      ArmIO io, DoubleSupplier voltageSupplier, double motorRotationsAtMaxAngle) {
    this(io, voltageSupplier, () -> Constants.Arm.HOLD_VOLTAGE, motorRotationsAtMaxAngle);
  }

  public ArmSubsystem(
      ArmIO io,
      DoubleSupplier voltageSupplier,
      DoubleSupplier holdVoltageSupplier,
      double motorRotationsAtMaxAngle) {
    this(
        io,
        voltageSupplier,
        holdVoltageSupplier,
        Constants.Arm.TARGET_ANGLE_DEGREES,
        motorRotationsAtMaxAngle);
  }

  public ArmSubsystem(
      ArmIO io,
      DoubleSupplier voltageSupplier,
      DoubleSupplier holdVoltageSupplier,
      double targetAngleDegrees,
      double motorRotationsAtMaxAngle) {
    this(
        io,
        voltageSupplier,
        holdVoltageSupplier,
        targetAngleDegrees,
        motorRotationsAtMaxAngle,
        () -> 0.0,
        () -> 0.0);
  }

  public ArmSubsystem(
      ArmIO io,
      DoubleSupplier voltageSupplier,
      DoubleSupplier holdVoltageSupplier,
      double motorRotationsAtMaxAngle,
      DoubleSupplier maxSpeedDegreesPerSecondSupplier,
      DoubleSupplier voltageSlewVoltsPerSecondSupplier) {
    this(
        io,
        voltageSupplier,
        holdVoltageSupplier,
        Constants.Arm.TARGET_ANGLE_DEGREES,
        motorRotationsAtMaxAngle,
        maxSpeedDegreesPerSecondSupplier,
        voltageSlewVoltsPerSecondSupplier);
  }

  public ArmSubsystem(
      ArmIO io,
      DoubleSupplier voltageSupplier,
      DoubleSupplier holdVoltageSupplier,
      double targetAngleDegrees,
      double motorRotationsAtMaxAngle,
      DoubleSupplier maxSpeedDegreesPerSecondSupplier,
      DoubleSupplier voltageSlewVoltsPerSecondSupplier) {
    this.io = io;
    this.voltageSupplier = voltageSupplier;
    this.holdVoltageSupplier = holdVoltageSupplier;
    this.maxSpeedDegreesPerSecondSupplier =
        fallbackWhenInvalid(
            maxSpeedDegreesPerSecondSupplier,
            Constants.Arm.DEFAULT_MAX_SPEED_DEGREES_PER_SECOND);
    this.voltageSlewVoltsPerSecondSupplier =
        fallbackWhenInvalid(
            voltageSlewVoltsPerSecondSupplier,
            Constants.Arm.DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND);
    this.targetAngleDegrees =
        MathUtil.clamp(
            targetAngleDegrees, Constants.Arm.MIN_ANGLE_DEGREES, Constants.Arm.MAX_ANGLE_DEGREES);
    this.motorRotationsAtMaxAngle =
        Double.isFinite(motorRotationsAtMaxAngle)
                && Math.abs(motorRotationsAtMaxAngle) >= MINIMUM_VALID_CALIBRATION_ROTATIONS
            ? OptionalDouble.of(motorRotationsAtMaxAngle)
            : OptionalDouble.empty();
    encoderPositions = io.getEncoderPositions();
    previousAngleDegrees = getAngleDegrees(encoderPositions);
    holdSetpointDegrees = previousAngleDegrees;
    filteredHoldAngleDegrees = previousAngleDegrees;
    previousTimestampSeconds = Timer.getFPGATimestamp();
    lastMovementTimestampSeconds = previousTimestampSeconds;
  }

  public double getAverageMotorRotations() {
    return encoderPositions.averageRotations();
  }

  public double getAngleDegrees() {
    return getAngleDegrees(encoderPositions);
  }

  public boolean isCalibrated() {
    return motorRotationsAtMaxAngle.isPresent();
  }

  /**
   * True once the arm has reached its upper limit, and stays true while the arm rests near the
   * top (the fade hold settles slightly below 90 degrees). Clears once the arm drops back
   * below the unlock threshold. Gates the elevator commands.
   */
  public boolean isRaised() {
    return reachedUpperLimit;
  }

  public void zeroEncoders() {
    stop();
    io.zeroEncoders();
    encoderPositions = new ArmEncoderPositions(0.0, 0.0);
  }

  public void moveUp() {
    if (!isCalibrated() || atTargetAngle()) {
      stop();
      return;
    }
    applyMovementVoltage(targetAngleDegrees, true);
  }

  public void moveDown() {
    if (!isCalibrated() || atLowerLimit()) {
      stop();
      return;
    }
    double voltageMagnitude =
        Math.min(
            getRequestedVoltageMagnitude(),
            descentVoltageLimit(getAngleDegrees() - Constants.Arm.MIN_ANGLE_DEGREES));
    double targetVoltage =
        -Math.copySign(voltageMagnitude, motorRotationsAtMaxAngle.getAsDouble());
    applyRawMovementVoltage(targetVoltage, false);
  }

  /**
   * Moves one step toward the middle working angle (button 5's destination), from whichever
   * side the arm is on; the direction is latched when the enclosing command starts. Unlike
   * the descent to the bottom limit, the approach keeps the 0.8V floor in both directions:
   * mid-travel is not a gravity-settling rest, so the arm must keep enough authority to stop
   * on the angle instead of coasting through it. If the whole tolerance window is somehow
   * skipped in one cycle, the command does not reverse into an endless oscillation: it keeps
   * its latched direction and the far hard limit ends the run.
   */
  public void moveTowardMidAngle() {
    if (!isCalibrated()
        || atMidAngle()
        || (midMovementIsUpward ? atUpperLimit() : atLowerLimit())) {
      stop();
      return;
    }
    // Between the start side and the mid angle the taper follows the mid angle; past it (the
    // tolerance window was skipped in one cycle) it follows the exit hard limit instead.
    double goalAngleDegrees =
        midMovementIsUpward
            ? (getAngleDegrees() < Constants.Arm.MID_ANGLE_DEGREES
                ? Constants.Arm.MID_ANGLE_DEGREES
                : Constants.Arm.MAX_ANGLE_DEGREES)
            : (getAngleDegrees() > Constants.Arm.MID_ANGLE_DEGREES
                ? Constants.Arm.MID_ANGLE_DEGREES
                : Constants.Arm.MIN_ANGLE_DEGREES);
    applyMovementVoltage(goalAngleDegrees, midMovementIsUpward);
  }

  /** Steps toward an arbitrary goal angle using the floored approach taper. */
  private void applyMovementVoltage(double goalAngleDegrees, boolean upward) {
    double distanceDegrees = Math.abs(goalAngleDegrees - getAngleDegrees());
    double voltageMagnitude =
        Math.min(getRequestedVoltageMagnitude(), approachVoltageLimit(distanceDegrees));
    double targetVoltage =
        upward
            ? Math.copySign(voltageMagnitude, motorRotationsAtMaxAngle.getAsDouble())
            : -Math.copySign(voltageMagnitude, motorRotationsAtMaxAngle.getAsDouble());
    applyRawMovementVoltage(targetVoltage, upward);
  }

  /**
   * Applies a movement voltage through the spec's safety ceilings: first the acceleration limit
   * (the voltage ramps at most the configured volts per second from the last movement voltage),
   * then the speed limit (the voltage is cut to zero when the arm already moves at the cap in
   * the commanded direction). Both limits are NetworkTables-tunable; 0 or non-finite disables.
   */
  private void applyRawMovementVoltage(double targetVoltage, boolean upward) {
    double timestampSeconds = Timer.getFPGATimestamp();
    double dtSeconds = timestampSeconds - lastMovementTimestampSeconds;
    double slewedVoltage =
        slewTowards(
            targetVoltage,
            lastMovementVoltage,
            dtSeconds,
            readPositiveSupplier(voltageSlewVoltsPerSecondSupplier));
    double velocityInMotionDirection =
        upward ? angleVelocityDegreesPerSecond : -angleVelocityDegreesPerSecond;
    double limitedVoltage =
        voltageAfterSpeedCap(
            slewedVoltage,
            velocityInMotionDirection,
            readPositiveSupplier(maxSpeedDegreesPerSecondSupplier));
    lastMovementVoltage = limitedVoltage;
    lastMovementTimestampSeconds = timestampSeconds;
    applyVoltage(limitedVoltage);
  }

  static double slewTowards(
      double targetVoltage, double currentVoltage, double dtSeconds, double voltsPerSecond) {
    if (!Double.isFinite(voltsPerSecond) || voltsPerSecond <= 0.0) {
      return targetVoltage;
    }
    if (dtSeconds <= 0.0) {
      return currentVoltage;
    }
    double maxDeltaVolts = voltsPerSecond * dtSeconds;
    return currentVoltage
        + MathUtil.clamp(targetVoltage - currentVoltage, -maxDeltaVolts, maxDeltaVolts);
  }

  static double voltageAfterSpeedCap(
      double voltage, double velocityInMotionDirection, double maxSpeed) {
    if (!Double.isFinite(maxSpeed) || maxSpeed <= 0.0) {
      return voltage;
    }
    return velocityInMotionDirection >= maxSpeed ? 0.0 : voltage;
  }

  private static double readPositiveSupplier(DoubleSupplier supplier) {
    double value = supplier.getAsDouble();
    return Double.isFinite(value) ? Math.max(value, 0.0) : 0.0;
  }

  /**
   * Wraps a NetworkTables limit so an invalid reading (non-finite or negative — a mistyped
   * dashboard value) falls back to the validated default instead of silently disabling the
   * safety ceiling. Zero remains a deliberate "limit off" switch.
   */
  private static DoubleSupplier fallbackWhenInvalid(DoubleSupplier supplier, double fallback) {
    return () -> {
      double value = supplier.getAsDouble();
      return Double.isFinite(value) && value >= 0.0 ? value : fallback;
    };
  }

  private static double approachVoltageLimit(double degreesFromLimit) {
    double approachFraction =
        MathUtil.clamp(degreesFromLimit / Constants.Arm.APPROACH_SLOWDOWN_DEGREES, 0.0, 1.0);
    return Constants.Arm.APPROACH_VOLTAGE
        + approachFraction * (Constants.Arm.MAX_VOLTAGE - Constants.Arm.APPROACH_VOLTAGE);
  }

  private static double descentVoltageLimit(double degreesFromLimit) {
    // Descending, gravity pulls the arm toward the lower limit, so the taper ends at 0V:
    // pushing any voltage into the limit adds energy and slams the arm past zero.
    double approachFraction =
        MathUtil.clamp(degreesFromLimit / Constants.Arm.APPROACH_SLOWDOWN_DEGREES, 0.0, 1.0);
    return approachFraction * Constants.Arm.MAX_VOLTAGE;
  }

  public boolean atUpperLimit() {
    return isCalibrated() && getAngleDegrees() >= Constants.Arm.MAX_ANGLE_DEGREES;
  }

  /** True when the arm has reached its configured target angle (button 4's destination). */
  public boolean atTargetAngle() {
    return isCalibrated() && getAngleDegrees() >= targetAngleDegrees;
  }

  /** True when the arm is within the tolerance window of the middle working angle. */
  public boolean atMidAngle() {
    return isCalibrated()
        && Math.abs(getAngleDegrees() - Constants.Arm.MID_ANGLE_DEGREES)
            <= Constants.Arm.MID_ANGLE_TOLERANCE_DEGREES;
  }

  public boolean atLowerLimit() {
    return isCalibrated() && getAngleDegrees() <= Constants.Arm.MIN_ANGLE_DEGREES;
  }

  /** Seconds the last movement command ran before finishing (or being interrupted), for the log. */
  public double getLastMoveDurationSeconds() {
    return lastMoveDurationSeconds;
  }

  /** Name of the last movement command that ran, e.g. "ArmToUpperTarget". */
  public String getLastMoveActionName() {
    return lastMoveActionName;
  }

  /**
   * Moves the arm up on plain voltage (smooth on this mechanism), stopping at the configured
   * target angle (default 90°, spec allows 85°). The voltage tapers near the target so the arm
   * arrives gently and the default hold command can catch it without arrival chatter.
   */
  public Command moveToUpperTargetCommand() {
    return timedMovementCommand(
        "ArmToUpperTarget", this::moveUp, () -> !isCalibrated() || atTargetAngle());
  }

  public Command moveToLowerLimitCommand() {
    return timedMovementCommand(
        "ArmToLowerLimit", this::moveDown, () -> !isCalibrated() || atLowerLimit());
  }

  /**
   * Moves the arm to the middle working angle from whichever side it is on, stopping inside
   * the tolerance window (the default hold command then latches the angle). The direction is
   * latched at start, and the far hard limit — the one actually on the path — ends the
   * command if the tolerance window is skipped.
   */
  public Command moveToMidAngleCommand() {
    return timedMovementCommand(
        "ArmToMidAngle",
        this::moveTowardMidAngle,
        () ->
            !isCalibrated()
                || atMidAngle()
                || (midMovementIsUpward ? atUpperLimit() : atLowerLimit()));
  }

  private Command timedMovementCommand(
      String name, Runnable moveStep, java.util.function.BooleanSupplier finishedCondition) {
    return new FunctionalCommand(
        () -> {
          lastMoveActionName = name;
          moveStartTimestampSeconds = Timer.getFPGATimestamp();
          midMovementIsUpward = getAngleDegrees() < Constants.Arm.MID_ANGLE_DEGREES;
        },
        moveStep,
        interrupted -> {
          stop();
          lastMoveDurationSeconds = Timer.getFPGATimestamp() - moveStartTimestampSeconds;
          System.out.printf(
              "Arm action %s took %.2f s%n", lastMoveActionName, lastMoveDurationSeconds);
        },
        finishedCondition,
        this)
        .withName(name);
  }

  /**
   * Latches the current angle on start and holds it in one of three zones: near the target, a
   * continuous gravity-compensation voltage fading to zero at the target (zero closing speed,
   * so it cannot buzz against the limit wall); mid-range, a one-way hysteresis nudge that
   * pushes the arm back after it sags past a deadband and releases before overshooting; at the
   * bottom, a gentle lean on the software zero from below — gravity already holds the arm
   * there, so no active fighting is needed.
   */
  public Command holdCurrentPositionCommand() {
    return Commands.startRun(this::latchHoldTarget, this::updateHoldVoltage, this)
        .withName("ArmHoldPosition");
  }

  public void stop() {
    // Reset the ramp reference so the next movement accelerates from zero, matching how the
    // movement commands always start from a stopped arm.
    lastMovementVoltage = 0.0;
    lastMovementTimestampSeconds = Timer.getFPGATimestamp();
    applyVoltage(0.0);
  }

  private double getRequestedVoltageMagnitude() {
    double requestedVoltage = Math.abs(voltageSupplier.getAsDouble());
    return Double.isFinite(requestedVoltage)
        ? Math.min(requestedVoltage, Constants.Arm.MAX_VOLTAGE)
        : 0.0;
  }

  private void latchHoldTarget() {
    holdSetpointDegrees =
        MathUtil.clamp(
            getAngleDegrees(), Constants.Arm.MIN_ANGLE_DEGREES, Constants.Arm.MAX_ANGLE_DEGREES);
    holdNudgeActive = false;
    filteredHoldAngleDegrees = getAngleDegrees();
    // The hold is not a movement: the next movement command must ramp from zero, not from
    // whatever the hold was applying when it was interrupted.
    lastMovementVoltage = 0.0;
  }

  private void updateHoldVoltage() {
    if (!isCalibrated()) {
      stop();
      return;
    }
    double errorDegrees = holdSetpointDegrees - getAngleDegrees();
    // At the bottom of travel gravity itself holds the arm (it naturally rests slightly below
    // the calibrated zero), so nudging there would fight gravity forever and buzz in place.
    boolean gravityHoldsTheArm =
        holdSetpointDegrees
            <= Constants.Arm.MIN_ANGLE_DEGREES + Constants.Arm.HOLD_ERROR_DEADBAND_DEGREES;
    // Hysteresis: start nudging when the arm sags more than the deadband below the latch point,
    // keep nudging until it is back within half the deadband. The arm therefore drifts down
    // slowly, gets a gentle fixed-voltage push back up, and never overshoots the latch point —
    // no two-sided feedback to oscillate against the software limit wall.
    if (gravityHoldsTheArm) {
      holdNudgeActive = false;
    } else if (errorDegrees > Constants.Arm.HOLD_ERROR_DEADBAND_DEGREES) {
      holdNudgeActive = true;
    } else if (errorDegrees < Constants.Arm.HOLD_ERROR_DEADBAND_DEGREES * 0.5) {
      holdNudgeActive = false;
    }
    double outputVoltage = 0.0;
    if (holdSetpointDegrees
        >= targetAngleDegrees - Constants.Arm.UPPER_HOLD_ZONE_OFFSET_DEGREES) {
      // High latches hold with a continuous gravity-compensation voltage that fades to zero
      // below the target, so the arm lands on it with zero closing speed instead of bouncing
      // past it. The 2 degree band keeps the effective gain gentle enough that encoder
      // quantization cannot hunt around the equilibrium.
      double topFadeDegrees =
          MathUtil.clamp(
              (targetAngleDegrees - filteredHoldAngleDegrees)
                  / Constants.Arm.TOP_FADE_DEGREES,
              0.0,
              1.0);
      outputVoltage =
          MathUtil.clamp(holdVoltageSupplier.getAsDouble(), 0.0, Constants.Arm.MAX_VOLTAGE)
              * topFadeDegrees;
    } else if (holdNudgeActive) {
      outputVoltage =
          Math.min(getRequestedVoltageMagnitude(), Constants.Arm.HOLD_NUDGE_VOLTAGE);
    } else if (gravityHoldsTheArm) {
      // The arm's physical rest sits slightly below the calibrated zero, so at the bottom it
      // leans on the software zero from below with a gentle voltage that fades to zero over
      // the last degree — the exact mirror of the top fade, for a soft landing at 0°.
      double belowZeroDegrees = Constants.Arm.MIN_ANGLE_DEGREES - filteredHoldAngleDegrees;
      double bottomFade =
          MathUtil.clamp(
              belowZeroDegrees / Constants.Arm.BOTTOM_FADE_DEGREES, 0.0, 1.0);
      outputVoltage = Constants.Arm.BOTTOM_HOLD_VOLTAGE * bottomFade;
    }
    // Software limit wall: never push the arm further past a limit it is already at.
    if (atUpperLimit()) {
      outputVoltage = Math.min(outputVoltage, 0.0);
    }
    if (atLowerLimit()) {
      outputVoltage = Math.max(outputVoltage, 0.0);
    }
    applyVoltage(outputVoltage * Math.signum(motorRotationsAtMaxAngle.getAsDouble()));
    // The hold is not a movement, but it must keep the movement clock current: without this,
    // the first movement after a long hold sees a huge dt and skips the voltage ramp.
    lastMovementTimestampSeconds = Timer.getFPGATimestamp();
  }

  private void applyVoltage(double voltage) {
    appliedVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    encoderPositions = io.getEncoderPositions();
    double currentAngleDegrees = getAngleDegrees(encoderPositions);
    // Elevator interlock: reaching the target angle grants permission; dropping well below it
    // (the arm leaving the top) revokes it.
    if (currentAngleDegrees >= targetAngleDegrees) {
      reachedUpperLimit = true;
    } else if (currentAngleDegrees
        < targetAngleDegrees - Constants.Arm.ELEVATOR_UNLOCK_MARGIN_DEGREES) {
      reachedUpperLimit = false;
    }
    double timestampSeconds = Timer.getFPGATimestamp();
    double dtSeconds = timestampSeconds - previousTimestampSeconds;
    double rawVelocityDegreesPerSecond =
        isCalibrated() && dtSeconds > MINIMUM_VELOCITY_TIMESTEP_SECONDS
            ? (currentAngleDegrees - previousAngleDegrees) / dtSeconds
            : 0.0;
    // Low-pass the finite-difference velocity: encoder quantization (about 0.2 degrees per
    // rotor count) otherwise produces per-cycle spikes in the telemetry trace.
    angleVelocityDegreesPerSecond =
        VELOCITY_FILTER_ALPHA * rawVelocityDegreesPerSecond
            + (1.0 - VELOCITY_FILTER_ALPHA) * angleVelocityDegreesPerSecond;
    // Light low-pass on the angle used by the hold fades: encoder quantization (about 0.2
    // degrees) otherwise dithers the fade output and the arm hunts ±0.5 degrees at the top.
    filteredHoldAngleDegrees =
        HOLD_ANGLE_FILTER_ALPHA * currentAngleDegrees
            + (1.0 - HOLD_ANGLE_FILTER_ALPHA) * filteredHoldAngleDegrees;
    previousAngleDegrees = currentAngleDegrees;
    previousTimestampSeconds = timestampSeconds;

    Logger.recordOutput("Arm/Motor1Rotations", encoderPositions.motor1Rotations(), "rotations");
    Logger.recordOutput("Arm/Motor2Rotations", encoderPositions.motor2Rotations(), "rotations");
    Logger.recordOutput(
        "Arm/AverageMotorRotations", encoderPositions.averageRotations(), "rotations");
    Logger.recordOutput(
        "Arm/EncoderDifferenceRotations", encoderPositions.differenceRotations(), "rotations");
    Logger.recordOutput("Arm/AngleDegrees", currentAngleDegrees, "degrees");
    Logger.recordOutput("Arm/AngleVelocityDegreesPerSecond", angleVelocityDegreesPerSecond, "deg/s");
    Logger.recordOutput("Arm/HoldSetpointDegrees", holdSetpointDegrees, "degrees");
    Logger.recordOutput("Arm/HoldFilteredAngleDegrees", filteredHoldAngleDegrees, "degrees");
    Logger.recordOutput("Arm/Calibrated", isCalibrated());
    Logger.recordOutput(
        "Arm/AtLowerLimit",
        isCalibrated()
            && getAngleDegrees(encoderPositions) <= Constants.Arm.MIN_ANGLE_DEGREES);
    Logger.recordOutput(
        "Arm/AtUpperLimit",
        isCalibrated()
            && getAngleDegrees(encoderPositions) >= Constants.Arm.MAX_ANGLE_DEGREES);
    Logger.recordOutput("Arm/RequestedVoltage", getRequestedVoltageMagnitude(), "volts");
    Logger.recordOutput("Arm/AppliedVoltage", appliedVoltage, "volts");
    Logger.recordOutput(
        "Arm/MotorRotationsAtMaxAngle", motorRotationsAtMaxAngle.orElse(0.0), "rotations");
    Logger.recordOutput("Arm/LastMoveDurationSeconds", lastMoveDurationSeconds, "seconds");
    Logger.recordOutput("Arm/LastMoveActionName", lastMoveActionName);
  }

  private double getAngleDegrees(ArmEncoderPositions positions) {
    if (!isCalibrated()) {
      return 0.0;
    }
    double normalizedPosition =
        positions.averageRotations() / motorRotationsAtMaxAngle.getAsDouble();
    return Constants.Arm.MIN_ANGLE_DEGREES
        + normalizedPosition
            * (Constants.Arm.MAX_ANGLE_DEGREES - Constants.Arm.MIN_ANGLE_DEGREES);
  }
}
