// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

/**
 * ArmIO variant that reads the arm angle from an absolute through-bore encoder on the shaft
 * instead of averaging the two TalonFX rotor positions. Voltages still drive both arm motors.
 *
 * <p>The encoder constants in {@link Constants.Arm} are placeholders: fill in the DIO channel,
 * gear ratio, and zero offset when the hardware arrives, then flip {@code
 * USE_THROUGH_BORE_ENCODER} in {@code RobotContainer} to switch the angle source. The rest of
 * the subsystem is unchanged — the absolute angle is presented through the existing
 * "average motor rotations" calibration math, so it round-trips exactly.
 */
public class ArmIOThroughBore implements ArmSubsystem.ArmIO {
  private final TalonFX motor1 = new TalonFX(Constants.Arm.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Arm.MOTOR_2_ID);
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(Constants.Arm.THROUGH_BORE_DIO_CHANNEL);

  public ArmIOThroughBore() {
    MotorOutputConfigs config =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(
                Constants.Arm.MOTOR_1_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive);
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public ArmSubsystem.ArmEncoderPositions getEncoderPositions() {
    // Absolute encoder: output is a 0..1 fraction of a full encoder turn. Convert to an arm
    // angle (accounting for mounting gearing and the mechanical-zero offset) and normalize
    // into [0, 360) so a shaft mounted slightly past the wrap point still reads correctly.
    double angleDegrees =
        MathUtil.inputModulus(
            encoder.get() * 360.0 / Constants.Arm.THROUGH_BORE_GEAR_RATIO
                + Constants.Arm.THROUGH_BORE_OFFSET_DEGREES,
            0.0,
            360.0);
    // Present the angle through the existing calibration math so no subsystem code changes:
    // rotations = angle / travel * rotations-at-max-travel.
    double rotations =
        angleDegrees
            / (Constants.Arm.MAX_ANGLE_DEGREES - Constants.Arm.MIN_ANGLE_DEGREES)
            * Constants.Arm.MOTOR_ROTATIONS_AT_MAX_ANGLE;
    return new ArmSubsystem.ArmEncoderPositions(rotations, rotations);
  }

  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  @Override
  public void zeroEncoders() {
    // Absolute encoder: nothing to zero at startup. The offset constant defines the
    // mechanical zero, which removes the "arm must be at zero before power-on" requirement.
  }
}
