package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmSubsystem.ArmIO {
  private final TalonFX motor1 = new TalonFX(Constants.Arm.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Arm.MOTOR_2_ID);

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
  public double getMotor1Rotations() {
    return motor1.getPosition().refresh().getValueAsDouble();
  }

  @Override
  public double getMotor2Rotations() {
    return motor2.getPosition().refresh().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  @Override
  public void zeroEncoders() {
    motor1.setPosition(0.0);
    motor2.setPosition(0.0);
  }
}
