// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmIOTalonFX;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorIOTalonFX;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final LoggedNetworkNumber armVoltage =
      new LoggedNetworkNumber("/SmartDashboard/Arm Voltage", Constants.Arm.DEFAULT_VOLTAGE);

  private final ArmSubsystem armSubsystem =
      new ArmSubsystem(
          new ArmIOTalonFX(), armVoltage, Constants.Arm.MOTOR_ROTATIONS_AT_MAX_ANGLE);
  private final ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem(new ElevatorIOTalonFX());

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem.zeroEncoders();
    elevatorSubsystem.zeroEncoders();
    configureBindings();
  }

  private void configureBindings() {
    // This controller reports Xbox-style raw button IDs in Driver Station: Y=4 and A=1.
    m_driverController
        .button(OperatorConstants.kArmUpButton)
        .onTrue(armSubsystem.moveToUpperLimitCommand());
    m_driverController
        .button(OperatorConstants.kArmDownButton)
        .onTrue(armSubsystem.moveToLowerLimitCommand());
    m_driverController
        .button(OperatorConstants.kElevatorExtendButton)
        .onTrue(elevatorSubsystem.moveToUpperLimitCommand());
    m_driverController
        .button(OperatorConstants.kElevatorRetractButton)
        .onTrue(elevatorSubsystem.moveToLowerLimitCommand());
  }
}
