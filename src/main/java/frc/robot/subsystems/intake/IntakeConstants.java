// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  public static final double rollerMaxVolts = 8.0;
  public static final Rotation2d extendedAngle = Rotation2d.fromDegrees(120.0);
  public static final Rotation2d retractedAngle = Rotation2d.fromDegrees(0.0);

  public static final double rollerGearRatio = (1.0 / 5.0);
  public static final double pivotGearRatio = (2.0 * Math.PI) * (1.0 / 3.37) * (1.0 / 5.0);

  /** The constants only for the real version of the intake. */
  public static class Real {

    public static final int rollerMotorID = 18;
    public static final int pivotMotorID = 40;

    public static final SparkMaxConfig rollerConfig;
    public static final SparkMaxConfig pivotConfig;

    static {
      rollerConfig = new SparkMaxConfig();

      rollerConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).inverted(true);

      rollerConfig
          .encoder
          .positionConversionFactor(rollerGearRatio)
          .velocityConversionFactor(rollerGearRatio);

      rollerConfig.closedLoop.pid(2.0, 0.0, 0.0);

      pivotConfig = new SparkMaxConfig();

      pivotConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).inverted(false);

      pivotConfig
          .encoder
          .positionConversionFactor(pivotGearRatio)
          .velocityConversionFactor(pivotGearRatio / 60.0);

      pivotConfig.closedLoop.pid(1.0, 0.0, 0.0).allowedClosedLoopError(1.0, ClosedLoopSlot.kSlot0);

      pivotConfig
          .softLimit
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true)
          .forwardSoftLimit(1.5)
          .reverseSoftLimit(0.0);
    }
  }

  /** The constants only for the sim version of the intake. */
  public static class Sim {

    public static final DCMotor motorGearBox = DCMotor.getNEO(1);
    public static final double JKgMetersSquared = 0.004;
  }
}
