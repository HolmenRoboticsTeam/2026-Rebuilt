// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The turret interface for its inputs and outputs. */
public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    // Rotation motor
    public double rotationPositionRad = 0.0;
    public double rotationVelocityRadPerSec = 0.0;
    public double rotationAppliedVolts = 0.0;
    public double rotationCurrentAmps = 0.0;
    public boolean rotationIsAtTarget = false;

    // Angle motor
    public double anglePositionRad = 0.0;
    public double angleVelocityRadPerSec = 0.0;
    public double angleAppliedVolts = 0.0;
    public double angleCurrentAmps = 0.0;
    public boolean angleIsAtTarget = false;

    // FlyWheel motors
    public double flyWheelPositionRotations = 0.0;
    public double flyWheelVelocityRPM = 0.0;
    public double flyWheelAppliedVolts = 0.0;
    public double flyWheelCurrentAmps = 0.0;
    public boolean flyWheelIsTarget = false;
  }

  /**
   * Updates the inputs for the implementation of the turret.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Sets the target rotation of the turret's rotation motor. This does not need to be called every
   * cycle, as it runs on the controller.
   *
   * @param rot The target rotation.
   */
  public default void setTargetRotation(Rotation2d rot) {}

  /**
   * Sets the target angle of the turret's angle motor. This does not need to be called every cycle,
   * as it runs on the controller.
   *
   * @param angle The target angle.
   */
  public default void setTargetAngle(Rotation2d angle) {}

  /**
   * Sets the target RPM of the turret's flyWheel motors. This does not need to be called every
   * cycle, as it runs on the controller.
   *
   * @param rot The target RPM.
   */
  public default void setFlyWheelRPM(double RPM) {}

  /**
   * Sets the target rotation of the turret's rotation motor to ZERO. This does not need to be
   * called every cycle, as it runs on the controller.
   *
   * @param lockRotation Whether to lock or unlock the turret's rotation
   */
  public default void lockRotation(boolean lockRotation) {}

  /**
   * Shoots fuel based on the robot current state (Only used for sim).
   *
   * @param robotRotation the rotation of the drive base.
   */
  public default void shootFuel(Rotation2d robotRotation) {}
}
