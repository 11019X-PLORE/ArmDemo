package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  @Test
  void reportsBothEncoderPositionsAsPositiveWhileExtending() {
    FakeElevatorIO io = new FakeElevatorIO();
    io.motor1Rotations = 6.0;
    io.motor2Rotations = -4.0;
    ElevatorSubsystem elevator = new ElevatorSubsystem(io);

    assertEquals(6.0, elevator.getMotor1ExtensionRotations(), 1e-9);
    assertEquals(4.0, elevator.getMotor2ExtensionRotations(), 1e-9);
    assertEquals(5.0, elevator.getAverageExtensionRotations(), 1e-9);
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
    assertEquals(0.0, elevator.getAverageExtensionRotations(), 1e-9);
  }

  private static class FakeElevatorIO implements ElevatorSubsystem.ElevatorIO {
    double motor1Rotations;
    double motor2Rotations;

    @Override
    public ElevatorSubsystem.ElevatorEncoderPositions getEncoderPositions() {
      return new ElevatorSubsystem.ElevatorEncoderPositions(motor1Rotations, motor2Rotations);
    }

    @Override
    public void zeroEncoders() {
      motor1Rotations = 0.0;
      motor2Rotations = 0.0;
    }
  }
}
