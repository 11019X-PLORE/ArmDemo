// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the dual-motor arm. Data flows one way per section: sensor values are
 * polled into {@link ArmIOInputs} exactly once per cycle by {@link #updateInputs}, and commands
 * (voltage, zeroing) flow back through the setters. Implementations hide everything vendor
 * specific — the subsystem only ever sees this interface.
 */
public interface ArmIO {
  /**
   * Reads every arm sensor once and fills {@code inputs}. Implementations should batch their
   * hardware reads into a single transaction so one cycle sees one consistent snapshot.
   */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Applies the same voltage to both arm motors. */
  default void setVoltage(double voltage) {}

  /** Defines the current position as the calibration zero. */
  default void zeroEncoders() {}

  /** Everything read from the arm hardware in one cycle; logged wholesale for replay. */
  @AutoLog
  class ArmIOInputs {
    public double motor1Rotations;
    public double motor2Rotations;
    /** Voltage of the most recent setVoltage() call, reported back so the log records it. */
    public double appliedVolts;
  }
}
