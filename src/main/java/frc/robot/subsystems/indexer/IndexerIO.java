// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** The indexer interface for the its inputs and outputs. */
public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double positionRotation = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hasFuel = false;
  }

  /**
   * Updates the inputs for the implementation of the indexer.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /**
   * Sets the voltage of the indexer motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setVolts(double volts) {}
}
