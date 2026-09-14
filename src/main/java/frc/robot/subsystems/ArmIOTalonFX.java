// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** ArmIO implementation for the real TalonFX hardware (CAN IDs from {@link Constants.Arm}). */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX motor1 = new TalonFX(Constants.Arm.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Arm.MOTOR_2_ID);
  private final StatusSignal<Angle> motor1Position = motor1.getPosition();
  private final StatusSignal<Angle> motor2Position = motor2.getPosition();
  private double lastAppliedVolts;

  public ArmIOTalonFX() {
    configureMotor(motor1, Constants.Arm.MOTOR_1_INVERTED);
    configureMotor(motor2, Constants.Arm.MOTOR_2_INVERTED);
  }

  private static void configureMotor(TalonFX motor, boolean inverted) {
    MotorOutputConfigs config =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(
                inverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive);
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ArmIO.ArmIOInputs inputs) {
    // One CAN transaction refreshes every signal the subsystem consumes this cycle.
    BaseStatusSignal.refreshAll(motor1Position, motor2Position);
    inputs.motor1Rotations = motor1Position.getValueAsDouble();
    inputs.motor2Rotations = motor2Position.getValueAsDouble();
    inputs.appliedVolts = lastAppliedVolts;
  }

  @Override
  public void setVoltage(double voltage) {
    lastAppliedVolts = voltage;
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  @Override
  public void zeroEncoders() {
    motor1.setPosition(0.0);
    motor2.setPosition(0.0);
  }
}
