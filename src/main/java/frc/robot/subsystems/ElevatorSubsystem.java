package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  public record ElevatorEncoderPositions(double motor1Rotations, double motor2Rotations) {
    public double motor1ExtensionRotations() {
      return motor1Rotations;
    }

    public double motor2ExtensionRotations() {
      return -motor2Rotations;
    }

    public double averageExtensionRotations() {
      return (motor1ExtensionRotations() + motor2ExtensionRotations()) / 2.0;
    }

    public double differenceRotations() {
      return Math.abs(motor1ExtensionRotations() - motor2ExtensionRotations());
    }
  }

  public interface ElevatorIO {
    ElevatorEncoderPositions getEncoderPositions();

    void zeroEncoders();
  }

  private final ElevatorIO io;
  private ElevatorEncoderPositions encoderPositions;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    encoderPositions = io.getEncoderPositions();
  }

  public double getMotor1ExtensionRotations() {
    return encoderPositions.motor1ExtensionRotations();
  }

  public double getMotor2ExtensionRotations() {
    return encoderPositions.motor2ExtensionRotations();
  }

  public double getAverageExtensionRotations() {
    return encoderPositions.averageExtensionRotations();
  }

  public void zeroEncoders() {
    io.zeroEncoders();
    encoderPositions = new ElevatorEncoderPositions(0.0, 0.0);
  }

  @Override
  public void periodic() {
    encoderPositions = io.getEncoderPositions();

    Logger.recordOutput(
        "Elevator/Motor1ExtensionRotations",
        encoderPositions.motor1ExtensionRotations(),
        "rotations");
    Logger.recordOutput(
        "Elevator/Motor2ExtensionRotations",
        encoderPositions.motor2ExtensionRotations(),
        "rotations");
    Logger.recordOutput(
        "Elevator/AverageExtensionRotations",
        encoderPositions.averageExtensionRotations(),
        "rotations");
    Logger.recordOutput(
        "Elevator/EncoderDifferenceRotations",
        encoderPositions.differenceRotations(),
        "rotations");
    Logger.recordOutput(
        "Elevator/RawMotor1Rotations", encoderPositions.motor1Rotations(), "rotations");
    Logger.recordOutput(
        "Elevator/RawMotor2Rotations", encoderPositions.motor2Rotations(), "rotations");
  }
}
