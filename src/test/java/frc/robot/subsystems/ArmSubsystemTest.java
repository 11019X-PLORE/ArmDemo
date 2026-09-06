// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class ArmSubsystemTest {
  @Test
  void calculatesAngleFromTheAverageEncoderPosition() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 10.0;
    io.motor2Rotations = 30.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 1.0, 40.0);

    assertEquals(45.0, arm.getAngleDegrees(), 1e-9);
  }

  @Test
  void preventsMovementUntilTheArmIsCalibrated() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 10.0;
    io.motor2Rotations = 12.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 1.0, 0.0);

    arm.moveUp();

    assertFalse(arm.isCalibrated());
    assertEquals(0.0, arm.getAngleDegrees(), 1e-9);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void zeroesBothEncodersAtTheKnownArmZeroPosition() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 7.0;
    io.motor2Rotations = 9.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 1.0, 40.0);

    arm.zeroEncoders();

    assertEquals(0.0, io.motor1Rotations, 1e-9);
    assertEquals(0.0, io.motor2Rotations, 1e-9);
    assertEquals(0.0, arm.getAngleDegrees(), 1e-9);
  }

  @Test
  void stopsPositiveVoltageAtTheUpperAngleLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 39.0;
    io.motor2Rotations = 41.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);

    arm.moveUp();

    assertTrue(arm.atUpperLimit());
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void stopsNegativeVoltageAtTheLowerAngleLimit() {
    FakeArmIO io = new FakeArmIO();
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);

    arm.moveDown();

    assertTrue(arm.atLowerLimit());
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void appliesTheRequestedThreeVoltsInBothDirections() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);

    arm.moveUp();
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    arm.moveDown();
    assertEquals(-3.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void clampsDashboardVoltageToThreeVolts() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 12.0, 40.0);

    arm.moveUp();
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    arm.moveDown();
    assertEquals(-3.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void followsTheMeasuredEncoderDirectionWhenCalibrationIsNegative() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = -20.0;
    io.motor2Rotations = -20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, -40.0);

    arm.moveUp();
    assertEquals(-2.0, io.appliedVoltage, 1e-9);

    arm.moveDown();
    assertEquals(2.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void upperLimitCommandRunsOnVoltageUntilNinetyDegreesAndThenStops() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToUpperTargetCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void lowerLimitCommandRunsOnVoltageUntilZeroDegreesAndThenStops() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToLowerLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-3.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 0.0;
    io.motor2Rotations = 0.0;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void limitCommandsFinishImmediatelyWhenTheArmIsAlreadyAtTheTarget() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToUpperTargetCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
    assertTrue(command.isFinished());
  }

  @Test
  void upperTargetCommandCanStopAtAConfiguredAngle() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, () -> 2.4, 85.0, 40.0);
    Command command = arm.moveToUpperTargetCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    // 85.005 degrees: the configured target, so the command finishes and the elevator
    // interlock unlocks even though the arm is below the hard 90 degree limit.
    io.motor1Rotations = 37.78;
    io.motor2Rotations = 37.78;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
    assertTrue(arm.isRaised());
  }

  @Test
  void moveUpTapersVoltageNearTheUpperLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 37.5;
    io.motor2Rotations = 37.5;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);

    arm.moveUp();

    // 84.375 degrees leaves 5.625 degrees to the limit: 0.8 + (5.625 / 15) * 2.2.
    assertEquals(1.625, io.appliedVoltage, 1e-9);
  }

  @Test
  void moveDownTapersVoltageToZeroNearTheLowerLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 2.5;
    io.motor2Rotations = 2.5;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);

    arm.moveDown();

    // 5.625 degrees from the limit: gravity finishes the approach, so the taper runs all
    // the way to 0V at the limit instead of keeping a floor.
    assertEquals(-1.125, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandNudgesTheArmBackUpWhenItSagsBeyondTheDeadband() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    command.initialize();

    io.motor1Rotations = 18.0;
    io.motor2Rotations = 18.0;
    arm.periodic();
    command.execute();

    assertEquals(1.6, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandAppliesAContinuousVoltageWhenLatchedHigh() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 37.7;
    io.motor2Rotations = 37.7;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();

    // 83.25 degrees is above the upper hold zone and outside the top fade band: the hold
    // outputs the full continuous gravity-compensation voltage.
    io.motor1Rotations = 37.0;
    io.motor2Rotations = 37.0;
    arm.periodic();
    command.execute();

    assertEquals(2.4, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandFadesItsVoltageSmoothlyIntoTheUpperLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 39.75;
    io.motor2Rotations = 39.75;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    // 89.4375 degrees: 0.5625 degrees below the limit, attenuated by the remaining fraction
    // of the 2 degree fade band: 2.4 * (0.5625 / 2.0).
    assertEquals(0.675, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandUsesHysteresisAroundTheDeadband() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    command.initialize();

    // 43.875 degrees: 1.125 below the latch point, past the deadband, so the nudge starts.
    io.motor1Rotations = 19.5;
    io.motor2Rotations = 19.5;
    arm.periodic();
    command.execute();
    assertEquals(1.6, io.appliedVoltage, 1e-9);

    // 44.28 degrees: back inside the deadband, but the nudge stays on until the error is
    // within half the deadband.
    io.motor1Rotations = 19.68;
    io.motor2Rotations = 19.68;
    arm.periodic();
    command.execute();
    assertEquals(1.6, io.appliedVoltage, 1e-9);

    // 44.775 degrees: error 0.225 is inside the release threshold, so the nudge stops.
    io.motor1Rotations = 19.9;
    io.motor2Rotations = 19.9;
    arm.periodic();
    command.execute();
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandDoesNotPushWhenTheArmIsAboveTheLatchPoint() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 45.0;
    io.motor2Rotations = 45.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandNeverDrivesPastTheUpperSoftwareLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 41.0;
    io.motor2Rotations = 41.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandLeansOnTheZeroPositionFromBelowAtTheLowerLimit() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = -2.0;
    io.motor2Rotations = -2.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    // The arm's rest sits below the calibrated zero: while it is below zero the hold
    // leans it back on the software zero with the gentle bottom voltage.
    assertEquals(0.6, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandFadesItsBottomVoltageNearZero() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = -0.2;
    io.motor2Rotations = -0.2;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    // Half a degree into the fade band the voltage is attenuated by the remaining
    // distance: 0.6 * (0.45 / 1.0).
    assertEquals(0.27, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandIsQuietAtOrAboveZero() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 0.5;
    io.motor2Rotations = 0.5;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    arm.periodic();
    command.initialize();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandStopsWhenTheArmIsNotCalibrated() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 10.0;
    io.motor2Rotations = 12.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 0.0);
    Command command = arm.holdCurrentPositionCommand();

    command.initialize();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandRespectsTheMeasuredEncoderDirection() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = -20.0;
    io.motor2Rotations = -20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, -40.0);
    Command command = arm.holdCurrentPositionCommand();

    command.initialize();

    io.motor1Rotations = -18.0;
    io.motor2Rotations = -18.0;
    arm.periodic();
    command.execute();

    assertEquals(-1.6, io.appliedVoltage, 1e-9);
  }

  @Test
  void holdCommandOutputsNothingInsideTheDeadband() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 19.8;
    io.motor2Rotations = 19.8;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.holdCurrentPositionCommand();

    command.initialize();

    io.motor1Rotations = 19.7;
    io.motor2Rotations = 19.7;
    arm.periodic();
    command.execute();

    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void elevatorInterlockUnlocksAtTheUpperLimitAndClearsWhenLowered() {
    FakeArmIO io = new FakeArmIO();
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    assertFalse(arm.isRaised());

    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    arm.periodic();
    assertTrue(arm.isRaised());

    // Resting slightly below 90 (the fade hold) keeps the interlock engaged.
    io.motor1Rotations = 39.0;
    io.motor2Rotations = 39.0;
    arm.periodic();
    assertTrue(arm.isRaised());

    io.motor1Rotations = 10.0;
    io.motor2Rotations = 10.0;
    arm.periodic();
    assertFalse(arm.isRaised());
  }

  @Test
  void slewRampsTheVoltageAtMostTheConfiguredRate() {
    // 60 V/s over a 20 ms cycle moves the reference at most 1.2 V.
    assertEquals(1.2, ArmSubsystem.slewTowards(3.0, 0.0, 0.02, 60.0), 1e-9);
    assertEquals(-2.2, ArmSubsystem.slewTowards(-3.0, -1.0, 0.02, 60.0), 1e-9);
    // A rate of 0 (or anything non-positive) disables the ramp.
    assertEquals(3.0, ArmSubsystem.slewTowards(3.0, 0.0, 0.02, 0.0), 1e-9);
    // Already at the target: nothing to ramp.
    assertEquals(3.0, ArmSubsystem.slewTowards(3.0, 3.0, 0.02, 60.0), 1e-9);
  }

  @Test
  void speedCapCutsTheVoltageOnlyInTheTooFastDirection() {
    // The velocity is oriented along the commanded motion (positive = moving the commanded way).
    assertEquals(0.0, ArmSubsystem.voltageAfterSpeedCap(3.0, 60.0, 60.0), 1e-9);
    assertEquals(3.0, ArmSubsystem.voltageAfterSpeedCap(3.0, 59.9, 60.0), 1e-9);
    // Descending at the cap is vetoed even though the voltage sign is negative.
    assertEquals(0.0, ArmSubsystem.voltageAfterSpeedCap(-3.0, 60.0, 60.0), 1e-9);
    assertEquals(-3.0, ArmSubsystem.voltageAfterSpeedCap(-3.0, 59.9, 60.0), 1e-9);
    // Moving against the commanded direction is never the ceiling's business.
    assertEquals(-3.0, ArmSubsystem.voltageAfterSpeedCap(-3.0, -60.0, 60.0), 1e-9);
    // A cap of 0 (or anything non-positive) disables the ceiling.
    assertEquals(3.0, ArmSubsystem.voltageAfterSpeedCap(3.0, 1000.0, 0.0), 1e-9);
  }

  @Test
  void moveUpCutsTheVoltageWhenTheArmAlreadyMovesAtTheSpeedCeiling() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, () -> 2.4, 40.0, () -> 10.0, () -> 0.0);

    arm.angleVelocityDegreesPerSecond = 12.0;
    arm.moveUp();
    assertEquals(0.0, io.appliedVoltage, 1e-9);

    arm.angleVelocityDegreesPerSecond = 8.0;
    arm.moveUp();
    assertEquals(3.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void moveDownCutsTheVoltageWhenDescendingAtTheSpeedCeiling() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, () -> 2.4, 40.0, () -> 10.0, () -> 0.0);

    arm.angleVelocityDegreesPerSecond = -12.0;
    arm.moveDown();
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void nonFiniteTuningValuesFallBackToTheDefaults() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm =
        new ArmSubsystem(
            io, () -> 3.0, () -> 2.4, 40.0, () -> Double.NaN, () -> Double.POSITIVE_INFINITY);

    // A mistyped NetworkTables value must not silently disable the ceilings: the speed cap
    // falls back to its 60 deg/s default, so 500 deg/s is still cut to zero.
    arm.angleVelocityDegreesPerSecond = 500.0;
    arm.moveUp();
    assertEquals(0.0, io.appliedVoltage, 1e-9);

    // The slew falls back to its 60 V/s default: called microseconds after the previous
    // cycle, the ramp has barely moved, so the voltage stays near zero instead of jumping
    // straight to the full request.
    arm.angleVelocityDegreesPerSecond = 0.0;
    arm.moveUp();
    assertTrue(io.appliedVoltage < 0.5);
  }

  @Test
  void midCommandMovesDownFromAboveAndStopsInsideTheMidWindow() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToMidAngleCommand();

    arm.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-3.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    arm.periodic();
    assertTrue(arm.atMidAngle());
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
    assertEquals("ArmToMidAngle", arm.getLastMoveActionName());
    assertTrue(arm.getLastMoveDurationSeconds() >= 0.0);
  }

  @Test
  void midCommandMovesUpFromBelowAndStopsInsideTheMidWindow() {
    FakeArmIO io = new FakeArmIO();
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToMidAngleCommand();

    arm.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void midApproachKeepsTheFloorVoltageWhenDescendingTowardMid() {
    FakeArmIO io = new FakeArmIO();
    // 47.5 degrees: 2.5 degrees above mid, inside the 15 degree approach band. The descent to
    // mid keeps the 0.8V floor (unlike the descent to the bottom limit): 0.8 + (2.5/15) * 2.2.
    io.motor1Rotations = 21.1111;
    io.motor2Rotations = 21.1111;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);

    arm.periodic();
    arm.moveTowardMidAngle();

    assertEquals(-1.1667, io.appliedVoltage, 1e-3);
  }

  @Test
  void midCommandFallsBackToTheFarLimitWhenItSkipsTheMidWindow() {
    FakeArmIO io = new FakeArmIO();
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToMidAngleCommand();

    arm.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    // 48 degrees: the arm skipped the whole 43.5-46.5 window within one cycle. The
    // direction was latched upward at start, so the command keeps rising instead of
    // reversing into an oscillation, and the taper follows the upper hard limit now.
    io.motor1Rotations = 21.3333;
    io.motor2Rotations = 21.3333;
    arm.periodic();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(3.0, io.appliedVoltage, 1e-9);

    // The upper hard limit is the exit when the mid window is unreachable.
    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void firstMovementAfterALongHoldStillRampsFromZero() throws InterruptedException {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, () -> 2.4, 40.0, () -> 0.0, () -> 10.0);
    Command hold = arm.holdCurrentPositionCommand();

    arm.moveUp();
    arm.stop();
    hold.initialize();
    hold.execute();
    Thread.sleep(300);
    hold.execute();

    // The hold keeps the movement clock current, so the first movement after it ramps from
    // zero instead of seeing a 300 ms gap and jumping straight to full voltage.
    arm.moveUp();
    assertTrue(io.appliedVoltage < 0.5);
  }

  @Test
  void movementCommandsRecordTheirDurationAndName() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 3.0, 40.0);
    Command command = arm.moveToUpperTargetCommand();

    command.initialize();
    command.execute();

    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    arm.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);

    assertEquals("ArmToUpperTarget", arm.getLastMoveActionName());
    assertTrue(arm.getLastMoveDurationSeconds() >= 0.0);
  }

  private static class FakeArmIO implements ArmSubsystem.ArmIO {
    double motor1Rotations;
    double motor2Rotations;
    double appliedVoltage;

    @Override
    public ArmSubsystem.ArmEncoderPositions getEncoderPositions() {
      return new ArmSubsystem.ArmEncoderPositions(motor1Rotations, motor2Rotations);
    }

    @Override
    public void setVoltage(double voltage) {
      appliedVoltage = voltage;
    }

    @Override
    public void zeroEncoders() {
      motor1Rotations = 0.0;
      motor2Rotations = 0.0;
    }
  }
}
