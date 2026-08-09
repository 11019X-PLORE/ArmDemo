// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int ARM_UP_BUTTON = 4;
    public static final int ARM_DOWN_BUTTON = 2;
    public static final int ELEVATOR_EXTEND_BUTTON = 3;
    public static final int ELEVATOR_RETRACT_BUTTON = 1;

    private OperatorConstants() {}
  }

  public static final class Arm {
    public static final int MOTOR_1_ID = 1;
    public static final int MOTOR_2_ID = 2;

    public static final boolean MOTOR_1_INVERTED = false;
    public static final boolean MOTOR_2_INVERTED = false;

    public static final double DEFAULT_VOLTAGE = 3.0;
    public static final double MAX_VOLTAGE = 6.0;

    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;

    // Measured by manually moving the arm from 0 to 90 degrees and reading the average TalonFX
    // rotor position in AdvantageScope.
    public static final double MOTOR_ROTATIONS_AT_MAX_ANGLE = 0.2;

    private Arm() {}
  }

  public static final class Elevator {
    public static final int MOTOR_1_ID = 11;
    public static final int MOTOR_2_ID = 12;

    public static final double MOVEMENT_VOLTAGE = 3.0;
    public static final double MIN_EXTENSION_ROTATIONS = 0.0;
    public static final double MOTOR_1_MAX_EXTENSION_ROTATIONS = 3.4;
    public static final double MOTOR_2_MAX_EXTENSION_ROTATIONS = 14.5;

    private Elevator() {}
  }

  private Constants() {}
}
