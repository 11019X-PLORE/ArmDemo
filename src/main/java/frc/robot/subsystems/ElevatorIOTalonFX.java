// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorSubsystem.ElevatorIO {
  private final TalonFX motor1 = new TalonFX(Constants.Elevator.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Elevator.MOTOR_2_ID);
  private final StatusSignal<Angle> motor1Position = motor1.getPosition();
  private final StatusSignal<Angle> motor2Position = motor2.getPosition();

  public ElevatorIOTalonFX() {
    configureMotor(motor1);
    configureMotor(motor2);
  }

  private static void configureMotor(TalonFX motor) {
    MotorOutputConfigs config =
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(config);
  }

  @Override
  public ElevatorSubsystem.ElevatorEncoderPositions getEncoderPositions() {
    BaseStatusSignal.refreshAll(motor1Position, motor2Position);
    return new ElevatorSubsystem.ElevatorEncoderPositions(
        motor1Position.getValueAsDouble(), motor2Position.getValueAsDouble());
  }

  @Override
  public void setMotorVoltages(double motor1Voltage, double motor2Voltage) {
    motor1.setVoltage(motor1Voltage);
    motor2.setVoltage(motor2Voltage);
  }

  @Override
  public void zeroEncoders() {
    motor1.setPosition(0.0);
    motor2.setPosition(0.0);
  }
}
