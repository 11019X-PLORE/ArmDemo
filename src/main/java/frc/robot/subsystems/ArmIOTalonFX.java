package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmSubsystem.ArmIO {
  private final TalonFX motor1 = new TalonFX(Constants.Arm.MOTOR_1_ID);
  private final TalonFX motor2 = new TalonFX(Constants.Arm.MOTOR_2_ID);
  private final StatusSignal<Angle> motor1Position = motor1.getPosition();
  private final StatusSignal<Angle> motor2Position = motor2.getPosition();

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
  public ArmSubsystem.ArmEncoderPositions getEncoderPositions() {
    BaseStatusSignal.refreshAll(motor1Position, motor2Position);
    return new ArmSubsystem.ArmEncoderPositions(
        motor1Position.getValueAsDouble(), motor2Position.getValueAsDouble());
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
