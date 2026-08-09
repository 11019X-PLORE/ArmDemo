// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmIOTalonFX;
import frc.robot.subsystems.ArmSubsystem;
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

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem.zeroEncoders();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.cross().onTrue(armSubsystem.moveToUpperLimitCommand());
    m_driverController.circle().onTrue(armSubsystem.moveToLowerLimitCommand());
    m_driverController.L1().whileTrue(armSubsystem.motorOneDirectionTestCommand());
    m_driverController.R1().whileTrue(armSubsystem.motorTwoDirectionTestCommand());
  }
}
