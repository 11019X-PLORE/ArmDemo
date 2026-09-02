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
