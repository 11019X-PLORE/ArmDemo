// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  public record ElevatorEncoderPositions(double motor1Rotations, double motor2Rotations) {
    public double motor1ExtensionRotations() {
      return -motor1Rotations;
    }

    public double motor2ExtensionRotations() {
      return -motor2Rotations;
    }
  }

  public interface ElevatorIO {
    ElevatorEncoderPositions getEncoderPositions();

    void setMotorVoltages(double motor1Voltage, double motor2Voltage);

    void zeroEncoders();
  }

  private final ElevatorIO io;
  private ElevatorEncoderPositions encoderPositions;
  private double motor1AppliedVoltage;
  private double motor2AppliedVoltage;

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

  public boolean atUpperLimit() {
    return getMotor1ExtensionRotations()
            >= Constants.Elevator.MOTOR_1_MAX_EXTENSION_ROTATIONS
        || getMotor2ExtensionRotations()
            >= Constants.Elevator.MOTOR_2_MAX_EXTENSION_ROTATIONS;
  }

  public boolean atLowerLimit() {
    return getMotor1ExtensionRotations() <= Constants.Elevator.MIN_EXTENSION_ROTATIONS
        || getMotor2ExtensionRotations() <= Constants.Elevator.MIN_EXTENSION_ROTATIONS;
  }

  public void extend() {
    if (atUpperLimit()) {
      stop();
      return;
    }
    applyMotorVoltages(
        Constants.Elevator.MOVEMENT_VOLTAGE, -Constants.Elevator.MOVEMENT_VOLTAGE);
  }

  public void retract() {
    if (atLowerLimit()) {
      stop();
      return;
    }
    applyMotorVoltages(
        -Constants.Elevator.MOVEMENT_VOLTAGE, Constants.Elevator.MOVEMENT_VOLTAGE);
  }

  public void stop() {
    applyMotorVoltages(0.0, 0.0);
  }

  public Command moveToUpperLimitCommand() {
    return Commands.runEnd(this::extend, this::stop, this)
        .until(this::atUpperLimit)
        .withName("ElevatorToUpperLimit");
  }

  public Command moveToLowerLimitCommand() {
    return Commands.runEnd(this::retract, this::stop, this)
        .until(this::atLowerLimit)
        .withName("ElevatorToLowerLimit");
  }

  public void zeroEncoders() {
    stop();
    io.zeroEncoders();
    encoderPositions = new ElevatorEncoderPositions(0.0, 0.0);
  }

  private void applyMotorVoltages(double motor1Voltage, double motor2Voltage) {
    motor1AppliedVoltage = motor1Voltage;
    motor2AppliedVoltage = motor2Voltage;
    io.setMotorVoltages(motor1Voltage, motor2Voltage);
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
        "Elevator/RawMotor1Rotations", encoderPositions.motor1Rotations(), "rotations");
    Logger.recordOutput(
        "Elevator/RawMotor2Rotations", encoderPositions.motor2Rotations(), "rotations");
    Logger.recordOutput("Elevator/Motor1AppliedVoltage", motor1AppliedVoltage, "volts");
    Logger.recordOutput("Elevator/Motor2AppliedVoltage", motor2AppliedVoltage, "volts");
    Logger.recordOutput("Elevator/AtLowerLimit", atLowerLimit());
    Logger.recordOutput("Elevator/AtUpperLimit", atUpperLimit());
    Logger.recordOutput(
        "Elevator/Motor1MaxExtensionRotations",
        Constants.Elevator.MOTOR_1_MAX_EXTENSION_ROTATIONS,
        "rotations");
    Logger.recordOutput(
        "Elevator/Motor2MaxExtensionRotations",
        Constants.Elevator.MOTOR_2_MAX_EXTENSION_ROTATIONS,
        "rotations");
  }
}
