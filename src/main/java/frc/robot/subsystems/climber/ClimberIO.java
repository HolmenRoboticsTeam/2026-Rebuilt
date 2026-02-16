// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** The Climber interface for its inputs and outputs. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean hardStop = false;
  }

  /**
   * Updates the inputs for the implementation of the climber.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the voltage of the climber motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setVolts(double volts) {}

  /** Resets the position of the climber motor to the min value. */
  public default void resetPosition() {}
}
