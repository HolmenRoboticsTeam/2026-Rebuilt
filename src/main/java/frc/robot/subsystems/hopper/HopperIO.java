// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** The hopper interface for its inputs and outputs. */
public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /**
   * Updates the inputs for the implementation of the hopper.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(HopperIOInputs inputs) {}

  /**
   * Sets the voltage of the hopper motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setVolts(double volts) {}
}
