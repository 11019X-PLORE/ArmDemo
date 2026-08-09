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
  void clampsDashboardVoltageAndAppliesTheRequestedDirection() {
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
  void upperLimitCommandRunsUntilNinetyDegreesAndThenStops() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.moveToUpperLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(2.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 40.0;
    io.motor2Rotations = 40.0;
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  @Test
  void lowerLimitCommandRunsUntilZeroDegreesAndThenStops() {
    FakeArmIO io = new FakeArmIO();
    io.motor1Rotations = 20.0;
    io.motor2Rotations = 20.0;
    ArmSubsystem arm = new ArmSubsystem(io, () -> 2.0, 40.0);
    Command command = arm.moveToLowerLimitCommand();

    command.initialize();
    command.execute();
    assertFalse(command.isFinished());
    assertEquals(-2.0, io.appliedVoltage, 1e-9);

    io.motor1Rotations = 0.0;
    io.motor2Rotations = 0.0;
    command.execute();
    assertTrue(command.isFinished());
    command.end(false);
    assertEquals(0.0, io.appliedVoltage, 1e-9);
  }

  private static class FakeArmIO implements ArmSubsystem.ArmIO {
    double motor1Rotations;
    double motor2Rotations;
    double appliedVoltage;

    @Override
    public double getMotor1Rotations() {
      return motor1Rotations;
    }

    @Override
    public double getMotor2Rotations() {
      return motor2Rotations;
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
