// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the climber subsystem. */
public class ClimberConstants {

  public static final double retractedHeight = 0.5;
  public static final double extendHeight = 0.76;

  public static final double gearRatio = 1.0 / 27.0;
  public static final double drumCircumference = 0.03 * Math.PI; // Meters
  public static final double motorToLengthRatio = gearRatio * drumCircumference;

  /** The constants only for the real version of the climber. */
  public static class Real {

    public static final int motorID = 27;
    public static final int limitSwitchID = 4;

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
    public static final double JKgMetersSquared = 0.00001;

    public static final double climberP = 20.0;
    public static final double climberI = 0.0;
    public static final double climberD = 0.0;
  }
}
