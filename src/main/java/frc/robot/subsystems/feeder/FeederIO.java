// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** The feeder interface for its inputs and outputs. */
public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hasEnterFuel = true;
    public boolean hasExitFuel = true;
    public boolean releasingFuel = false;
  }

  /**
   * Updates the inputs for the implementation of the feeder.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(FeederIOInputs inputs) {}

  /**
   * Sets the voltage of the feeder motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setVolts(double volts) {}
}
