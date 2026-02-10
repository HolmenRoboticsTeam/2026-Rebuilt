// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexIO {

  @AutoLog
  public static class IndexIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IndexIOInputsAutoLogged inputs) {}

  public default void setVoltage(double volts) {}
}
