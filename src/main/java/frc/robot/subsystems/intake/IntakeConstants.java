// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  public static final double maxVolts = 5.0;

  public static final double gearRatio = (2.0 * Math.PI) * (1.0);

  /** The constants only for the real version of the intake. */
  public static class Real {

    public static final int leftMotorID = 18;
    public static final int rightMotorID = 30;

    public static final SparkMaxConfig leftMotorConfig;
    public static final SparkMaxConfig rightMotorConfig;

    static {
      leftMotorConfig = new SparkMaxConfig();

      leftMotorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast).inverted(true);

      leftMotorConfig
          .encoder
          .positionConversionFactor(gearRatio)
          .velocityConversionFactor(gearRatio / 60.0);

      leftMotorConfig.closedLoop.pid(1.0, 0.0, 0.0);

      rightMotorConfig = new SparkMaxConfig();

      rightMotorConfig.follow(leftMotorID, true);
    }
  }

  /** The constants only for the sim version of the intake. */
  public static class Sim {

    public static final DCMotor motorGearBox = DCMotor.getNEO(1);
    public static final double JKgMetersSquared = 0.004;
  }
}
