// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  @Test
  void reportsBothEncoderPositionsAsPositiveWhileExtending() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -3.4;
    io.motor2Rotations = -14.5;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);

    assertEquals(3.4, elevator.getMotor1ExtensionRotations(), 1e-9);
    assertEquals(14.5, elevator.getMotor2ExtensionRotations(), 1e-9);
  }

  @Test
  void zeroesBothEncodersAtTheKnownMinimumLength() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 8.0;
    io.motor2Rotations = -8.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);

    elevator.zeroEncoders();

    assertEquals(0.0, io.motor1Rotations, 1e-9);
    assertEquals(0.0, io.motor2Rotations, 1e-9);
    assertEquals(0.0, elevator.getMotor1ExtensionRotations(), 1e-9);
    assertEquals(0.0, elevator.getMotor2ExtensionRotations(), 1e-9);
  }

  @Test
  void reachesTheUpperLimitWhenEitherMotorReachesItsOwnMaximum() {
    FakeElevatorIO motor1AtLimitIO = new FakeElevatorIO();
    motor1AtLimitIO.motor1Rotations = -3.4;
    motor1AtLimitIO.motor2Rotations = -10.0;
    ElevatorSubsystem elevator1 = new ElevatorSubsystem(motor1AtLimitIO);

    FakeElevatorIO motor2AtLimitIO = new FakeElevatorIO();
    motor2AtLimitIO.motor1Rotations = -2.0;
    motor2AtLimitIO.motor2Rotations = -14.5;
    ElevatorSubsystem elevator2 = new ElevatorSubsystem(motor2AtLimitIO);

    assertTrue(elevator1.atUpperLimit());
    assertTrue(elevator2.atUpperLimit());
  }

  @Test
  void appliesOppositeTwoVoltOutputsForExtensionAndRetraction() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);

    elevator.extend();
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    elevator.retract();
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void extensionCommandStopsBothMotorsWhenEitherUpperLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    Command command = elevator.moveToUpperLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    io.motor1Rotations = -3.4;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void retractionCommandStopsBothMotorsWhenEitherLowerLimitIsReached() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    Command command = elevator.moveToLowerLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);

    io.motor2Rotations = 0.0;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void slewRampsTheVoltageAtMostTheConfiguredRate() {
    assertEquals(1.2, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 60.0), 1e-9);
    assertEquals(0.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.0, 60.0), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.slewTowards(2.0, 0.0, 0.02, 0.0), 1e-9);
  }

  @Test
  void speedCapCutsTheVoltageOnlyInTheTooFastDirection() {
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.5, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 1.49, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, -10.0, 1.5), 1e-9);
    assertEquals(2.0, ElevatorSubsystem.voltageAfterSpeedCap(2.0, 10.0, 0.0), 1e-9);
    // Retracting at the cap (negative voltage) is cut the same way.
    assertEquals(0.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.5, 1.5), 1e-9);
    assertEquals(-2.0, ElevatorSubsystem.voltageAfterSpeedCap(-2.0, 1.49, 1.5), 1e-9);
  }

  @Test
  void nonFiniteSlewValuesFallBackToTheDefaultRamp() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, () -> Double.NaN, () -> Double.NaN);

    // A mistyped NetworkTables value must not disable the ramp: it falls back to the 60 V/s
    // default, and called microseconds after construction the ramp has barely moved.
    elevator.extend();
    assertTrue(io.motor1Voltage < 0.5);
    assertEquals(-io.motor1Voltage, io.motor2Voltage, 1e-9);
  }

  @Test
  void directionReversalsRampThroughZeroInsteadOfSnapping() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    double[] slewRate = {1.0e7};
    ElevatorSubsystem elevator = new ElevatorSubsystem(io, () -> 0.0, () -> slewRate[0]);

    // Build up to the full +2V with the ramp effectively disabled.
    elevator.extend();
    assertEquals(2.0, io.motor1Voltage, 0.1);

    // Then reverse with a slow 6 V/s ramp: the next call is microseconds later, so the
    // voltage must still be near +2V (heading for -2V through zero), not snapped to -2V.
    slewRate[0] = 6.0;
    elevator.retract();
    assertEquals(2.0, io.motor1Voltage, 0.1);
    assertEquals(-io.motor1Voltage, io.motor2Voltage, 1e-9);
  }

  @Test
  void midCommandExtendsToTheMiddleExtensionAndStops() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, 1e-9);
    assertEquals(-2.0, io.motor2Voltage, 1e-9);

    // Motor 1 reaching its own half travel (1.7 rotations) stops both motors.
    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atMidExtension());
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
    assertEquals("ElevatorToMidExtension", elevator.getLastMoveActionName());
    assertTrue(elevator.getLastMoveDurationSeconds() >= 0.0);
  }

  @Test
  void midCommandRetractsToTheMiddleExtensionFromAbove() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -3.4;
    io.motor2Rotations = -14.5;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-2.0, io.motor1Voltage, 1e-9);
    assertEquals(2.0, io.motor2Voltage, 1e-9);

    io.motor1Rotations = -1.7;
    elevator.periodic();
    assertTrue(elevator.atOrBelowMidExtension());
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void midCommandFinishesImmediatelyWhenOneMotorIsAlreadyPastItsOwnMid() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 0.0; // motor 1 fully retracted
    io.motor2Rotations = -7.25; // motor 2 exactly at its own mid extension
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);
    Command command = elevator.moveToMidExtensionCommand();

    elevator.periodic();
    command.initialize();
    command.execute();

    // The average travel fraction (0.25) latches extension, and motor 2 already satisfies
    // the "whichever mechanism arrives first" rule, so the command stops without moving.
    assertTrue(command.isFinished());
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
  }

  @Test
  void elevatorStopsAndEndsItsCommandWhenTheArmPermissionIsRevokedMidMotion() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    boolean[] permission = {true};
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(io, () -> permission[0], () -> 0.0, () -> 0.0);
    Command command = elevator.moveToUpperLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.motor1Voltage, 1e-9);

    // The arm drops below its unlock margin mid-motion (e.g. the arm-down button is
    // pressed): the next cycle must cut both motors and finish the command instead of
    // running to the hard limit with the arm down.
    permission[0] = false;
    command.execute();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
    assertTrue(command.isFinished());
  }

  @Test
  void elevatorRefusesToMoveWhileTheArmPermissionIsAbsent() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator =
        new ElevatorSubsystem(io, () -> false, () -> 0.0, () -> 0.0);

    elevator.extend();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    elevator.retract();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    elevator.moveTowardMidExtension();
    assertEquals(0.0, io.motor1Voltage, 1e-9);
  }

  private static class FakeElevatorIO implements ElevatorSubsystem.ElevatorIO {
    double motor1Rotations;
    double motor2Rotations;
    double motor1Voltage;
    double motor2Voltage;

    @Override
    public ElevatorSubsystem.ElevatorEncoderPositions getEncoderPositions() {
      return new ElevatorSubsystem.ElevatorEncoderPositions(motor1Rotations, motor2Rotations);
    }

    @Override
    public void zeroEncoders() {
      motor1Rotations = 0.0;
      motor2Rotations = 0.0;
    }

    @Override
    public void setMotorVoltages(double motor1Voltage, double motor2Voltage) {
      this.motor1Voltage = motor1Voltage;
      this.motor2Voltage = motor2Voltage;
    }
  }
}
