package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private static final double MINIMUM_VALID_CALIBRATION_ROTATIONS = 1e-6;

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

    void setMotorVoltages(double motor1Voltage, double motor2Voltage);

    default void setVoltage(double voltage) {
      setMotorVoltages(voltage, voltage);
    }

    void zeroEncoders();
  }

  private final ArmIO io;
  private final DoubleSupplier voltageSupplier;
  private final OptionalDouble motorRotationsAtMaxAngle;
  private ArmEncoderPositions encoderPositions;
  private double appliedVoltage;
  private double motor1AppliedVoltage;
  private double motor2AppliedVoltage;

  public ArmSubsystem(
      ArmIO io, DoubleSupplier voltageSupplier, double motorRotationsAtMaxAngle) {
    this.io = io;
    this.voltageSupplier = voltageSupplier;
    this.motorRotationsAtMaxAngle =
        Double.isFinite(motorRotationsAtMaxAngle)
                && Math.abs(motorRotationsAtMaxAngle) >= MINIMUM_VALID_CALIBRATION_ROTATIONS
            ? OptionalDouble.of(motorRotationsAtMaxAngle)
            : OptionalDouble.empty();
    encoderPositions = io.getEncoderPositions();
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

  public void zeroEncoders() {
    stop();
    io.zeroEncoders();
    encoderPositions = new ArmEncoderPositions(0.0, 0.0);
  }

  public void moveUp() {
    if (!isCalibrated() || atUpperLimit()) {
      stop();
      return;
    }
    applyVoltage(
        Math.copySign(getRequestedVoltageMagnitude(), motorRotationsAtMaxAngle.getAsDouble()));
  }

  public void moveDown() {
    if (!isCalibrated() || atLowerLimit()) {
      stop();
      return;
    }
    applyVoltage(
        -Math.copySign(getRequestedVoltageMagnitude(), motorRotationsAtMaxAngle.getAsDouble()));
  }

  public boolean atUpperLimit() {
    return isCalibrated() && getAngleDegrees() >= Constants.Arm.MAX_ANGLE_DEGREES;
  }

  public boolean atLowerLimit() {
    return isCalibrated() && getAngleDegrees() <= Constants.Arm.MIN_ANGLE_DEGREES;
  }

  public Command moveToUpperLimitCommand() {
    return Commands.runEnd(this::moveUp, this::stop, this)
        .until(() -> !isCalibrated() || atUpperLimit())
        .withName("ArmToUpperLimit");
  }

  public Command moveToLowerLimitCommand() {
    return Commands.runEnd(this::moveDown, this::stop, this)
        .until(() -> !isCalibrated() || atLowerLimit())
        .withName("ArmToLowerLimit");
  }

  public Command motorOneDirectionTestCommand() {
    return Commands.runEnd(
            () -> applyMotorVoltages(Constants.Arm.DIRECTION_TEST_VOLTAGE, 0.0),
            this::stop,
            this)
        .withName("ArmMotor1DirectionTest");
  }

  public Command motorTwoDirectionTestCommand() {
    return Commands.runEnd(
            () -> applyMotorVoltages(0.0, Constants.Arm.DIRECTION_TEST_VOLTAGE),
            this::stop,
            this)
        .withName("ArmMotor2DirectionTest");
  }

  public void stop() {
    applyVoltage(0.0);
  }

  private double getRequestedVoltageMagnitude() {
    double requestedVoltage = Math.abs(voltageSupplier.getAsDouble());
    return Double.isFinite(requestedVoltage)
        ? Math.min(requestedVoltage, Constants.Arm.MAX_TEST_VOLTAGE)
        : 0.0;
  }

  private void applyVoltage(double voltage) {
    applyMotorVoltages(voltage, voltage);
  }

  private void applyMotorVoltages(double motor1Voltage, double motor2Voltage) {
    motor1AppliedVoltage = motor1Voltage;
    motor2AppliedVoltage = motor2Voltage;
    appliedVoltage = (motor1Voltage + motor2Voltage) / 2.0;
    io.setMotorVoltages(motor1Voltage, motor2Voltage);
  }

  @Override
  public void periodic() {
    encoderPositions = io.getEncoderPositions();

    Logger.recordOutput("Arm/Motor1Rotations", encoderPositions.motor1Rotations(), "rotations");
    Logger.recordOutput("Arm/Motor2Rotations", encoderPositions.motor2Rotations(), "rotations");
    Logger.recordOutput(
        "Arm/AverageMotorRotations", encoderPositions.averageRotations(), "rotations");
    Logger.recordOutput(
        "Arm/EncoderDifferenceRotations", encoderPositions.differenceRotations(), "rotations");
    Logger.recordOutput("Arm/AngleDegrees", getAngleDegrees(encoderPositions), "degrees");
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
    Logger.recordOutput("Arm/Motor1AppliedVoltage", motor1AppliedVoltage, "volts");
    Logger.recordOutput("Arm/Motor2AppliedVoltage", motor2AppliedVoltage, "volts");
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
