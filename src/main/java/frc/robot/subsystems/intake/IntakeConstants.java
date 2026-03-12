// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  public static final double pivotMaxVolts = 6.0;
  public static final double rollersMaxVolts = 4.0;

  public static final double pivotGearRatio = (2.0 * Math.PI) * (1.0 / 1.0);
  public static final double rollersGearRatio = 1.0;

  public static final Rotation2d retractAngle = Rotation2d.fromDegrees(-45.0);
  public static final Rotation2d extendAngle = Rotation2d.fromDegrees(45);

  /** The constants only for the real version of the intake. */
  public static class Real {

    public static final int pivotMotorID = 30;
    public static final int rollerMotorID = 18;

    public static final SparkMaxConfig pivotMotorConfig;
    public static final SparkMaxConfig rollerMotorConfig;

    static {

      // Pivot config

      pivotMotorConfig = new SparkMaxConfig();

      pivotMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);

      pivotMotorConfig
          .encoder
          .positionConversionFactor(pivotGearRatio)
          .velocityConversionFactor(pivotGearRatio / 60.0); // Rads and Rads/sec

      pivotMotorConfig.closedLoop.pid(0.01, 0.0, 0.0);

      // Roller Config

      rollerMotorConfig = new SparkMaxConfig();

      rollerMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).inverted(false);

      rollerMotorConfig
          .encoder
          .positionConversionFactor(rollersGearRatio)
          .velocityConversionFactor(rollersGearRatio / 60.0); // Rotations and RPM

      rollerMotorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    }
  }

  /** The constants only for the sim version of the intake. */
  public static class Sim {

    public static final DCMotor pivotGearBox = DCMotor.getNEO(1);
    public static final double pivotJKgMetersSquared = 0.004;

    public static final double pivotP = 0.3;
    public static final double pivotI = 0.0;
    public static final double pivotD = 0.0;

    public static final DCMotor rollerGearBox = DCMotor.getNEO(1);
    public static final double rollerJKgMetersSquared = 0.004;
  }
}
