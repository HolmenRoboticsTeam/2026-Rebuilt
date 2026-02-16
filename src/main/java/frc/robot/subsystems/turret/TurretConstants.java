// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

/** Constants for the turret subsystem. */
public class TurretConstants {

  public static final Distance turretXOffset = Distance.ofRelativeUnits(0.0, Inch);
  public static final Distance turretYOffset = Distance.ofRelativeUnits(6.0, Inch);

  public static final double currentControlDebounce = 0.025;

  public static final double rotationTolerance = Math.toRadians(1.0);
  public static final double angleTolerance = Math.toRadians(1.0);
  public static final double flyWheelTolerance = 10.0; // RPM

  public static final double rotationGearing = 1.0 * (2.0 * Math.PI);
  public static final double angleGearing = 1.0 * (2.0 * Math.PI);
  public static final double flyWheelGearing = 1.0;

  /** The constants only for the real version of the turret. */
  public static class Real {

    public static final int rotationMotorID = 22;
    public static final int angleMotorID = 23;
    public static final int leftFlyWheelMotorID = 24;
    public static final int rightFlyWheelMotorID = 25;

    public static final double holdAmps = 20.0;

    public static final SparkFlexConfig rotationMotorConfig;
    public static final SparkMaxConfig angleMotorConfig;
    public static final SparkMaxConfig leftFlyWheelMotorConfig;
    public static final SparkMaxConfig rightFlyWheelMotorConfig;

    static {

      // Rotation Motor Config

      rotationMotorConfig = new SparkFlexConfig();

      rotationMotorConfig
          .encoder
          .positionConversionFactor(rotationGearing)
          .velocityConversionFactor(rotationGearing / 60.0);

      rotationMotorConfig.closedLoop.pid(0.01, 0.0, 0.0);

      // Angle Motor Config

      angleMotorConfig = new SparkMaxConfig();

      angleMotorConfig
          .encoder
          .positionConversionFactor(angleGearing)
          .velocityConversionFactor(angleGearing / 60.0);

      angleMotorConfig.closedLoop.pid(0.01, 0.0, 0.0);

      // Left FlyWheel Motor Config

      leftFlyWheelMotorConfig = new SparkMaxConfig();

      leftFlyWheelMotorConfig.inverted(false);

      leftFlyWheelMotorConfig
          .encoder
          .positionConversionFactor(flyWheelGearing)
          .velocityConversionFactor(flyWheelGearing);

      leftFlyWheelMotorConfig.closedLoop.pid(1.0, 0, 0);

      // Right FlyWheel Motor Config

      rightFlyWheelMotorConfig = new SparkMaxConfig();

      rightFlyWheelMotorConfig.inverted(true);

      rightFlyWheelMotorConfig
          .encoder
          .positionConversionFactor(flyWheelGearing)
          .velocityConversionFactor(flyWheelGearing);

      rightFlyWheelMotorConfig.closedLoop.pid(1.0, 0, 0);
    }
  }

  /** The constants only for the sim version of the turret. */
  public static class Sim {

    public static final DCMotor rotationMotorGearBox = DCMotor.getNeoVortex(1);
    public static final double rotationJKgMetersSquared = 0.004;
    public static final double rotationP = 0.01;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;

    public static final DCMotor angleMotorGearBox = DCMotor.getNEO(1);
    public static final double angleJKgMetersSquared = 0.004;
    public static final double angleP = 0.01;
    public static final double angleI = 0.0;
    public static final double angleD = 0.0;

    public static final DCMotor flyWheelMotorGearBox = DCMotor.getNEO(2);
    public static final double flyWheelJKgMetersSquared = 0.004;
    public static final double flyWheelP = 1.0;
    public static final double flyWheelI = 0.0;
    public static final double flyWheelD = 0.0;

    public static final double percentSpeedKept = 0.9;
    public static final double RPMToVelocityFactor = 0.02;
    public static final Distance turretHeight = Distance.ofRelativeUnits(20.0, Inch);
  }
}
