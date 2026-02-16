// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the climber subsystem. */
public class ClimberConstants {

  public static final double extendVolts = 12.0;
  public static final double retractVolts = -12.0;

  public static final double retractedHeight = 0.0;
  public static final double extendHeight = 30.0;

  public static final double gearRatio = 27.0;
  public static final double drumCircumference = 0.03 * Math.PI; // Meters
  public static final double motorToLengthRatio = gearRatio * drumCircumference;

  /** The constants only for the real version of the climber. */
  public static class Real {

    public static final int motorID = 27;
    public static final int limitSwitchID = 1;

    public static final SparkMaxConfig motorConfig;

    static {
      motorConfig = new SparkMaxConfig();

      motorConfig
          .encoder
          .positionConversionFactor(motorToLengthRatio)
          .velocityConversionFactor(motorToLengthRatio / 60.0);

      motorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    }
  }

  /** The constants only for for the sim version of the climber. */
  public static class Sim {

    public static final DCMotor motorGearBox = DCMotor.getNEO(1);
    public static final double JKgMetersSquared = 0.004;
  }
}
