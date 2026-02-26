// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

/** Constants for the turret subsystem. */
public class TurretConstants {

  public static final Distance turretXOffset = Distance.ofRelativeUnits(0.0, Inch);
  public static final Distance turretYOffset = Distance.ofRelativeUnits(-6.0, Inch);

  public static final double currentControlDebounce = 0.025;

  public static final double rotationTolerance = Math.toRadians(3.0);
  public static final double angleTolerance = Math.toRadians(5.0);
  public static final double flyWheelTolerance = 25.0; // RPM

  public static final double rotationGearing = (2.0 * Math.PI) * (1.0 / 3.0) * (10.0 / 88.0);
  public static final double angleGearing = (2.0 * Math.PI) * (15.0 / 36.0) * (10.0 / 192.0);
  public static final double flyWheelGearing = 1.0;

  /** The constants only for the real version of the turret. */
  public static class Real {

    public static final int rotationMotorID = 22;
    public static final int angleMotorID = 23;
    public static final int leftFlyWheelMotorID = 24;
    public static final int rightFlyWheelMotorID = 25;

    public static final double holdAmps = 30.0;

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

      rotationMotorConfig.inverted(true);

      rotationMotorConfig
          .closedLoop
          .pid(0.5, 0.0, 0.0)
          .outputRange(-0.2, 0.2)
          .allowedClosedLoopError(0.006, ClosedLoopSlot.kSlot0)
          .feedForward
          .kS(0.4);

      rotationMotorConfig
          .absoluteEncoder
          .zeroCentered(true)
          .zeroOffset(0.12281514)
          .positionConversionFactor(rotationGearing * 3.0);

      rotationMotorConfig
          .softLimit
          .forwardSoftLimit(Math.PI / 6)
          .reverseSoftLimit(-Math.PI / 6)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

      rotationMotorConfig.smartCurrentLimit(40);

      // Angle Motor Config

      angleMotorConfig = new SparkMaxConfig();

      angleMotorConfig
          .encoder
          .positionConversionFactor(angleGearing)
          .velocityConversionFactor(angleGearing / 60.0);

      angleMotorConfig
          .closedLoop
          .pid(0.6, 0.0, 0.0)
          .allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0)
          .feedForward
          .kS(0.25);
      angleMotorConfig.inverted(true);
      angleMotorConfig.softLimit.forwardSoftLimit(0.0).reverseSoftLimit(0.45);

      angleMotorConfig.smartCurrentLimit(40);

      // Left FlyWheel Motor Config

      leftFlyWheelMotorConfig = new SparkMaxConfig();

      leftFlyWheelMotorConfig.inverted(false);

      leftFlyWheelMotorConfig
          .encoder
          .positionConversionFactor(flyWheelGearing)
          .velocityConversionFactor(flyWheelGearing);

      leftFlyWheelMotorConfig
          .closedLoop
          .pid(0.0001, 0.0, 0.0)
          .allowedClosedLoopError(0.0, ClosedLoopSlot.kSlot0)
          .feedForward
          .kV(0.00192)
          .kS(0.4);

      leftFlyWheelMotorConfig.smartCurrentLimit(40);

      // Right FlyWheel Motor Config

      rightFlyWheelMotorConfig = new SparkMaxConfig();

      rightFlyWheelMotorConfig.smartCurrentLimit(40);

      rightFlyWheelMotorConfig.follow(leftFlyWheelMotorID, true);
    }
  }

  /** The constants only for the sim version of the turret. */
  public static class Sim {

    public static final DCMotor rotationMotorGearBox = DCMotor.getNeoVortex(1);
    public static final double rotationJKgMetersSquared = 0.04;
    public static final double rotationP = 5.0;
    public static final double rotationI = 0.0;
    public static final double rotationD = 3.0;

    public static final DCMotor angleMotorGearBox = DCMotor.getNEO(1);
    public static final double angleJKgMetersSquared = 0.004;
    public static final double angleP = 15.0;
    public static final double angleI = 0.0;
    public static final double angleD = 4.5;

    public static final DCMotor flyWheelMotorGearBox = DCMotor.getNEO(2);
    public static final double flyWheelJKgMetersSquared = 0.004;
    public static final double flyWheelP = 1.2;
    public static final double flyWheelI = 0.0;
    public static final double flyWheelD = 0.0;

    public static final double percentSpeedKept = 0.8;
    public static final double RPMToVelocityFactor = 0.0025;
    public static final Distance turretHeight = Distance.ofRelativeUnits(20.0, Inch);
  }
}
