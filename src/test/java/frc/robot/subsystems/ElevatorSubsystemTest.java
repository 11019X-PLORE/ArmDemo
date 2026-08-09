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
  void appliesOppositeOneVoltOutputsForExtensionAndRetraction() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = -1.0;
    io.motor2Rotations = -5.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);

    elevator.extend();
    assertEquals(1.0, io.motor1Voltage, 1e-9);
    assertEquals(-1.0, io.motor2Voltage, 1e-9);

    elevator.retract();
    assertEquals(-1.0, io.motor1Voltage, 1e-9);
    assertEquals(1.0, io.motor2Voltage, 1e-9);
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
    assertEquals(1.0, io.motor1Voltage, 1e-9);
    assertEquals(-1.0, io.motor2Voltage, 1e-9);

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
    assertEquals(-1.0, io.motor1Voltage, 1e-9);
    assertEquals(1.0, io.motor2Voltage, 1e-9);

    io.motor2Rotations = 0.0;
    elevator.periodic();
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.motor1Voltage, 1e-9);
    assertEquals(0.0, io.motor2Voltage, 1e-9);
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
