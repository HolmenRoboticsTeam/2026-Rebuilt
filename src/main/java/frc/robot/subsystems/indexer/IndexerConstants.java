// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the indexer subsystem. */
public class IndexerConstants {

  public static final double maxVolts = 4.0;

  public static final double gearRatio = 1.0;

  public static final double hasFuelDebouncerTime = 0.25;

  /** The constants only for the real version of the indexer. */
  public static class Real {

    public static final int motorID = 20;
    public static final int lineBreakID = 2;

    public static final SparkMaxConfig motorConfig;

    static {
      motorConfig = new SparkMaxConfig();

      motorConfig.idleMode(IdleMode.kCoast);
      motorConfig.inverted(true);

      motorConfig
          .encoder
          .positionConversionFactor(gearRatio)
          .velocityConversionFactor(gearRatio / 60.0);

      motorConfig.closedLoop.pid(1.0, 0.0, 0.0);

      motorConfig.smartCurrentLimit(40);
    }
  }

  /** The constants only for the sim version of the indexer. */
  public static class Sim {

    public static final DCMotor motorGearBox = DCMotor.getNEO(1);
    public static final double JKgMetersSquared = 0.004;

    public static final int maxHopperCapacity = 20;
  }
}
