package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private static final double MINIMUM_VALID_CALIBRATION_ROTATIONS = 1e-6;

  public interface ArmIO {
    double getMotor1Rotations();

    double getMotor2Rotations();

    void setVoltage(double voltage);

    void zeroEncoders();
  }

  private final ArmIO io;
  private final DoubleSupplier voltageSupplier;
  private final double motorRotationsAtMaxAngle;
  private double appliedVoltage;

  public ArmSubsystem(
      ArmIO io, DoubleSupplier voltageSupplier, double motorRotationsAtMaxAngle) {
    this.io = io;
    this.voltageSupplier = voltageSupplier;
    this.motorRotationsAtMaxAngle = motorRotationsAtMaxAngle;
  }

  public double getAverageMotorRotations() {
    return (io.getMotor1Rotations() + io.getMotor2Rotations()) / 2.0;
  }

  public double getAngleDegrees() {
    if (!isCalibrated()) {
      return 0.0;
    }
    return getAverageMotorRotations() / motorRotationsAtMaxAngle * 90.0;
  }

  public boolean isCalibrated() {
    return Math.abs(motorRotationsAtMaxAngle) >= MINIMUM_VALID_CALIBRATION_ROTATIONS;
  }

  public void zeroEncoders() {
    stop();
    io.zeroEncoders();
  }

  public void moveUp() {
    if (!isCalibrated() || atUpperLimit()) {
      stop();
      return;
    }
    applyVoltage(getRequestedVoltageMagnitude());
  }

  public void moveDown() {
    if (!isCalibrated() || atLowerLimit()) {
      stop();
      return;
    }
    applyVoltage(-getRequestedVoltageMagnitude());
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

  public void stop() {
    applyVoltage(0.0);
  }

  private double getRequestedVoltageMagnitude() {
    return Math.min(Math.abs(voltageSupplier.getAsDouble()), Constants.Arm.MAX_TEST_VOLTAGE);
  }

  private void applyVoltage(double voltage) {
    appliedVoltage = voltage;
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    double motor1Rotations = io.getMotor1Rotations();
    double motor2Rotations = io.getMotor2Rotations();

    Logger.recordOutput("Arm/Motor1Rotations", motor1Rotations, "rotations");
    Logger.recordOutput("Arm/Motor2Rotations", motor2Rotations, "rotations");
    Logger.recordOutput(
        "Arm/AverageMotorRotations", (motor1Rotations + motor2Rotations) / 2.0, "rotations");
    Logger.recordOutput(
        "Arm/EncoderDifferenceRotations",
        Math.abs(motor1Rotations - motor2Rotations),
        "rotations");
    Logger.recordOutput("Arm/AngleDegrees", getAngleDegrees(), "degrees");
    Logger.recordOutput("Arm/Calibrated", isCalibrated());
    Logger.recordOutput("Arm/AtLowerLimit", atLowerLimit());
    Logger.recordOutput("Arm/AtUpperLimit", atUpperLimit());
    Logger.recordOutput("Arm/RequestedVoltage", getRequestedVoltageMagnitude(), "volts");
    Logger.recordOutput("Arm/AppliedVoltage", appliedVoltage, "volts");
    Logger.recordOutput(
        "Arm/MotorRotationsAtMaxAngle", motorRotationsAtMaxAngle, "rotations");
  }
}
