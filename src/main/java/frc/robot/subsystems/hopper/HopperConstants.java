// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the hopper subsystem. */
public class HopperConstants {

  public static final double maxVolts = 12.0;

  public static final double gearRatio = 3.0;

  /** The constants only for the real version of the hopper. */
  public static class Real {

    public static final int motorID = 19;

    public static final SparkMaxConfig motorConfig;

    static {
      motorConfig = new SparkMaxConfig();

      motorConfig.idleMode(IdleMode.kCoast);

      motorConfig
          .encoder
          .positionConversionFactor(gearRatio)
          .velocityConversionFactor(gearRatio / 60.0);

      motorConfig.closedLoop.pid(1.0, 0.0, 0.0);
    }
  }

  /** The constants only for the sim version of the hopper. */
  public static class Sim {

    public static final DCMotor motorGearBox = DCMotor.getNEO(1);
    public static final double JKgMetersSquared = 0.004;
  }
}
