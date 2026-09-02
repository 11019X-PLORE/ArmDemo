// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final double targetAngleDegrees;
  private final OptionalDouble motorRotationsAtMaxAngle;
  private ArmEncoderPositions encoderPositions;
  private double appliedVoltage;
  private double holdSetpointDegrees;
  private boolean holdNudgeActive;
  private double previousAngleDegrees;
  private double previousTimestampSeconds;
  private double angleVelocityDegreesPerSecond;
  private double filteredHoldAngleDegrees;
  private boolean reachedUpperLimit;

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
    this.io = io;
    this.voltageSupplier = voltageSupplier;
    this.holdVoltageSupplier = holdVoltageSupplier;
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
    double voltageMagnitude =
        Math.min(
            getRequestedVoltageMagnitude(),
            approachVoltageLimit(targetAngleDegrees - getAngleDegrees()));
    applyVoltage(
        Math.copySign(voltageMagnitude, motorRotationsAtMaxAngle.getAsDouble()));
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
    applyVoltage(
        -Math.copySign(voltageMagnitude, motorRotationsAtMaxAngle.getAsDouble()));
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

  public boolean atLowerLimit() {
    return isCalibrated() && getAngleDegrees() <= Constants.Arm.MIN_ANGLE_DEGREES;
  }

  /**
   * Moves the arm up on plain voltage (smooth on this mechanism), stopping at the configured
   * target angle (default 90°, spec allows 85°). The voltage tapers near the target so the arm
   * arrives gently and the default hold command can catch it without arrival chatter.
   */
  public Command moveToUpperTargetCommand() {
    return Commands.runEnd(this::moveUp, this::stop, this)
        .until(() -> !isCalibrated() || atTargetAngle())
        .withName("ArmToUpperTarget");
  }

  public Command moveToLowerLimitCommand() {
    return Commands.runEnd(this::moveDown, this::stop, this)
        .until(() -> !isCalibrated() || atLowerLimit())
        .withName("ArmToLowerLimit");
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
