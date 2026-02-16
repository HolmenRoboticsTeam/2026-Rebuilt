// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** The intake interface for its inputs and outputs. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean isRunning = false;
  }

  /**
   * Updates the inputs for the implementation of the intake.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the voltage of the intake motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setVolts(double volts) {}
}
