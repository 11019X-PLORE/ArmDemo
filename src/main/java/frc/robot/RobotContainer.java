// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmIOThroughBore;
import frc.robot.subsystems.ArmIOTalonFX;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorIOTalonFX;
import frc.robot.subsystems.ElevatorSubsystem;
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
  private final LoggedNetworkNumber armHoldVoltage =
      new LoggedNetworkNumber("/SmartDashboard/Arm Hold Voltage", Constants.Arm.HOLD_VOLTAGE);

  private final ArmSubsystem armSubsystem =
      new ArmSubsystem(
          Constants.Arm.USE_THROUGH_BORE_ENCODER
              ? new ArmIOThroughBore()
              : new ArmIOTalonFX(),
          armVoltage,
          armHoldVoltage,
          Constants.Arm.MOTOR_ROTATIONS_AT_MAX_ANGLE);
  private final ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem(new ElevatorIOTalonFX());

  private final CommandPS5Controller driverController =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains the subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSubsystem.zeroEncoders();
    elevatorSubsystem.zeroEncoders();
    // Whenever no movement command is running, the arm actively holds its latched angle instead
    // of going limp and falling under gravity.
    armSubsystem.setDefaultCommand(armSubsystem.holdCurrentPositionCommand());
    configureBindings();
  }

  private void configureBindings() {
    // This controller reports Xbox-style raw button IDs in Driver Station.
    driverController
        .button(OperatorConstants.ARM_UP_BUTTON)
        .onTrue(armSubsystem.moveToUpperTargetCommand());
    driverController
        .button(OperatorConstants.ARM_DOWN_BUTTON)
        .onTrue(armSubsystem.moveToLowerLimitCommand());
    driverController
        .button(OperatorConstants.ELEVATOR_EXTEND_BUTTON)
        .onTrue(elevatorSubsystem.moveToUpperLimitCommand().onlyIf(armSubsystem::isRaised));
    driverController
        .button(OperatorConstants.ELEVATOR_RETRACT_BUTTON)
        .onTrue(elevatorSubsystem.moveToLowerLimitCommand().onlyIf(armSubsystem::isRaised));
  }
}
