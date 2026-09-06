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
    public static final int ARM_MID_BUTTON = 5;
    public static final int ELEVATOR_EXTEND_BUTTON = 3;
    public static final int ELEVATOR_RETRACT_BUTTON = 1;
    public static final int ELEVATOR_MID_BUTTON = 6;

    private OperatorConstants() {}
  }

  public static final class Arm {
    public static final int MOTOR_1_ID = 1;
    public static final int MOTOR_2_ID = 2;

    public static final boolean MOTOR_1_INVERTED = false;
    public static final boolean MOTOR_2_INVERTED = false;

    public static final double DEFAULT_VOLTAGE = 3.0;
    public static final double MAX_VOLTAGE = 3.0;

    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;

    // Button 4 raises the arm to and holds this angle (spec allows 90 or 85). The hard
    // software limit stays at MAX_ANGLE_DEGREES regardless of this setting.
    public static final double TARGET_ANGLE_DEGREES = 90.0;

    // Button 5 moves the arm to the middle working angle from whichever side it is on. The
    // approach keeps the 0.8V floor in both directions (unlike the bottom limit, mid-travel is
    // not a gravity-settling rest, so the arm must retain some downward authority to stop on it).
    public static final double MID_ANGLE_DEGREES = 45.0;
    public static final double MID_ANGLE_TOLERANCE_DEGREES = 1.5;

    // Safety ceilings for the spec's "settable max speed / max acceleration". The arm keeps its
    // validated open-loop voltage motion; these only bound it. Speed: when the filtered angle
    // velocity reaches the cap while moving, the movement voltage is cut to zero. Acceleration:
    // the movement voltage ramps at most this many volts per second. Both are live-tunable on
    // NetworkTables; a value of 0 (or non-finite) disables that limit. The 60 V/s default
    // reaches full voltage within about three 20 ms cycles, so it does not change the motion
    // the team validated.
    public static final double DEFAULT_MAX_SPEED_DEGREES_PER_SECOND = 60.0;
    public static final double DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND = 60.0;

    // Measured by manually moving the arm from 0 to 90 degrees and reading the average TalonFX
    // rotor position in AdvantageScope.
    public static final double MOTOR_ROTATIONS_AT_MAX_ANGLE = 0.2;

    // The elevator may only run after the arm has reached its target angle. The permission
    // clears when the arm drops this far back below the target (hysteresis, because the fade
    // hold rests slightly below the target).
    public static final double ELEVATOR_UNLOCK_MARGIN_DEGREES = 10.0;

    // Through-bore encoder placeholders: fill in the wiring when the hardware arrives, then
    // flip the flag to switch the arm's angle source from averaged TalonFX rotor positions
    // to the absolute shaft encoder.
    public static final boolean USE_THROUGH_BORE_ENCODER = false;
    public static final int THROUGH_BORE_DIO_CHANNEL = 0;
    public static final double THROUGH_BORE_GEAR_RATIO = 1.0; // encoder turns per arm turn
    public static final double THROUGH_BORE_OFFSET_DEGREES = 0.0; // mechanical zero alignment

    // Movement runs open-loop on voltage (smooth on this mechanism) but tapers the voltage
    // near each limit so the arm arrives gently and the hold command can catch it. Rising
    // keeps a 0.8V floor (gravity fights it); falling tapers fully to 0V because gravity
    // already pulls the arm toward the lower limit — any downward voltage there slams it
    // through zero (observed as a -22 degree overshoot).
    public static final double APPROACH_SLOWDOWN_DEGREES = 15.0;
    public static final double APPROACH_VOLTAGE = 0.8;

    // Voltage used to lean the arm on the software zero from below, fading to zero over the
    // last degree above the limit — the exact mirror of the top fade, so the arm lands on 0°
    // with zero closing speed instead of kicking against the on/off boundary.
    public static final double BOTTOM_HOLD_VOLTAGE = 0.6;
    public static final double BOTTOM_FADE_DEGREES = 1.0;

    // The hold command is a one-way hysteresis servo: when the arm sags more than this far
    // below its latched angle it is nudged back up at the nudge voltage, and the nudge
    // releases once it is within half the deadband again.
    public static final double HOLD_ERROR_DEADBAND_DEGREES = 1.0;
    public static final double HOLD_NUDGE_VOLTAGE = 1.6;

    // Above this latch angle the hold switches from hysteresis nudging to a continuous
    // gravity-compensation voltage that fades linearly to zero below 90°. The arm therefore
    // lands on the limit with zero closing speed: it never crosses the software wall, so there
    // is no on/off boundary to buzz against. The fade band is 2 degrees wide: a 1 degree band
    // made the effective gain (voltage / degree) steeper than the encoder quantization noise
    // could support, which hunted visibly around the equilibrium.
    public static final double UPPER_HOLD_ZONE_OFFSET_DEGREES = 15.0;
    public static final double HOLD_VOLTAGE = 2.4;
    public static final double TOP_FADE_DEGREES = 2.0;

    private Arm() {}
  }

  public static final class Elevator {
    public static final int MOTOR_1_ID = 11;
    public static final int MOTOR_2_ID = 12;

    public static final double MOVEMENT_VOLTAGE = 2.0;
    public static final double MIN_EXTENSION_ROTATIONS = 0.0;
    public static final double MOTOR_1_MAX_EXTENSION_ROTATIONS = 3.4;
    public static final double MOTOR_2_MAX_EXTENSION_ROTATIONS = 14.5;

    // Button 6 moves the elevator to the middle working extension from whichever side it is
    // on, expressed as a fraction of each motor's own travel so the two mechanisms (3.4 vs
    // 14.5 rotations) stay coordinated.
    public static final double MID_EXTENSION_FRACTION = 0.5;

    // Same spec ceilings as the arm, in elevator terms. Speed: each motor's extension is
    // normalized by its own travel, and when the faster motor's fraction-per-second reaches
    // the cap while moving, both movement voltages are cut to zero. Acceleration: movement
    // voltages ramp at most this many volts per second. Live-tunable on NetworkTables; 0 or
    // non-finite disables. The 60 V/s default is effectively instant at 2V and does not change
    // the validated motion.
    public static final double DEFAULT_MAX_TRAVEL_FRACTION_PER_SECOND = 1.5;
    public static final double DEFAULT_VOLTAGE_SLEW_VOLTS_PER_SECOND = 60.0;

    private Elevator() {}
  }

  private Constants() {}
}
