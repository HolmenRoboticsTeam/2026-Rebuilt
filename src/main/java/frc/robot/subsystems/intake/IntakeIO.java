// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The intake interface for its inputs and outputs. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollerPositionRotations = 0.0;
    public double rollerVelocityRPM = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  /**
   * Updates the inputs for the implementation of the intake.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the angle of the pivot motor. This does not need to be called every cycle, as it runs on
   * the controller.
   *
   * @param angle The target angle.
   */
  public default void setPivotAngle(Rotation2d angle) {}

  /**
   * Sets the voltage of the roller motor. This does not need to be called every cycle, as it runs
   * on the controller.
   *
   * @param volts The target voltage.
   */
  public default void setRollerVoltage(double volts) {}
}
