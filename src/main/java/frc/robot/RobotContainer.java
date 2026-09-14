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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (just the scheduler calls). Instead, the structure of the robot (including
 * subsystems, OI devices, and commands) should be declared here. Its job is wiring only: which
 * IO implementation backs each subsystem, how subsystems interconnect, and which buttons run
 * which commands. Every subsystem owns its own NetworkTables tuning entries and polls them in
 * its own periodic loop.
 */
public class RobotContainer {
  private final ArmSubsystem armSubsystem =
      new ArmSubsystem(
          Constants.Arm.USE_THROUGH_BORE_ENCODER
              ? new ArmIOThroughBore()
              : new ArmIOTalonFX(),
          Constants.Arm.MOTOR_ROTATIONS_AT_MAX_ANGLE);
  private final ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem(new ElevatorIOTalonFX(), armSubsystem::isRaised);

  private final CommandPS5Controller driverController =
      new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains the subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Whenever no movement command is running, the arm actively holds its latched angle instead
    // of going limp and falling under gravity.
    armSubsystem.setDefaultCommand(armSubsystem.holdCurrentPositionCommand());
    // Deliberately no startup zeroEncoders(): the TalonFX positions must survive a
    // robot-code restart, or a mid-match soft restart would re-zero wherever the
    // mechanisms happen to be and shift every software limit. After a full power cycle
    // the encoders read zero on their own, so the mechanisms must be at their physical
    // zero positions before power-on (see the README safety warning).
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
        .button(OperatorConstants.ARM_MID_BUTTON)
        .onTrue(armSubsystem.moveToMidAngleCommand());
    // The arm interlock runs on two layers: onlyIf gates the button press, and the elevator
    // subsystem re-checks armSubsystem::isRaised every cycle while moving — if the arm drops
    // below the unlock margin mid-motion (e.g. the arm-down button is pressed), the elevator
    // command ends immediately and both motors stop.
    driverController
        .button(OperatorConstants.ELEVATOR_EXTEND_BUTTON)
        .onTrue(elevatorSubsystem.moveToUpperLimitCommand().onlyIf(armSubsystem::isRaised));
    driverController
        .button(OperatorConstants.ELEVATOR_RETRACT_BUTTON)
        .onTrue(elevatorSubsystem.moveToLowerLimitCommand().onlyIf(armSubsystem::isRaised));
    driverController
        .button(OperatorConstants.ELEVATOR_MID_BUTTON)
        .onTrue(elevatorSubsystem.moveToMidExtensionCommand().onlyIf(armSubsystem::isRaised));
  }
}
