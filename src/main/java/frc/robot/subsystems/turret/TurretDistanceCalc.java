// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Add your docs here. */
public class TurretDistanceCalc {

  private static InverseInterpolator<Distance> inverseInterpolator =
      new InverseInterpolator<Distance>() {

        @Override
        public double inverseInterpolate(Distance startValue, Distance endValue, Distance q) {
          return MathUtil.inverseInterpolate(
              startValue.in(Meters), endValue.in(Meters), q.in(Meters));
        }
      };

  private static InterpolatingTreeMap<Distance, TurretShotData> hubMap =
      new InterpolatingTreeMap<Distance, TurretShotData>(
          inverseInterpolator, TurretDistanceCalc::interpolate);
  private static InterpolatingTreeMap<Distance, TurretShotData> groundMap =
      new InterpolatingTreeMap<Distance, TurretShotData>(
          inverseInterpolator, TurretDistanceCalc::interpolate);

  private static InterpolatingDoubleTreeMap velocityToEffectiveDistanceHubMap =
      new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap velocityToEffectiveDistanceGroundMap =
      new InterpolatingDoubleTreeMap();

  private static List<TurretShotData> hubDataPoints;
  private static List<TurretShotData> groundDataPoints;

  private static double minHubTimeOfFlight = 120.0;
  private static double maxHubTimeOfFlight = 0.0;

  /**
   * Looks up (or interpolates) the desired ShotData based on the type and distance of the target.
   *
   * @param type the type of target
   * @param distance the distance from the target
   * @return the desired ShotData
   */
  public static TurretShotData getShotData(TargetType type, Distance distance) {

    switch (type) {
      case HUB:
        return hubMap.get(distance);
      case GROUND:
        return groundMap.get(distance);
      case IN_TRENCH:
        // So that when the robot is under the trench the flywheel stays up to speed.
        TurretShotData unchangedValue = hubMap.get(distance);
        return new TurretShotData(
            unchangedValue.distance, unchangedValue.RPM, 0.0, unchangedValue.timeOfFlightSec);
      case INVALID:
        // Look at the hub to try to fix pose estimation
        return hubMap.get(distance);
    }

    return new TurretShotData(Meters.of(0.0), 0, 0, 0); // How?
  }

  /**
   * Using inverse interpolation, converts desired velocity to an effective distance for the shot.
   * This is not the actual distance the fuel will travel.
   *
   * @param velocity the desired velocity
   * @param type the type of target
   * @return the effective distance for the fuel.
   */
  public static double velocityToEffectiveDistance(double velocity, TargetType type) {

    switch (type) {
      case HUB:
        return velocityToEffectiveDistanceHubMap.get(velocity);
      case GROUND:
        return velocityToEffectiveDistanceGroundMap.get(velocity);
      case IN_TRENCH:
        return velocityToEffectiveDistanceHubMap.get(velocity);
      case INVALID:
        return velocityToEffectiveDistanceHubMap.get(velocity);
    }

    return 0.0; // How?
  }

  /**
   * Converts the given velocity to the corrected RPM.
   *
   * @param requiredVelocity the velocity to aim for.
   * @param type the type of target.
   * @return the corrected RPM.
   */
  public static double calculateAdjustedRpm(double requiredVelocity, TargetType type) {
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, type);
    return getShotData(type, Meter.of(effectiveDistance)).RPM;
  }

  /**
   * @return The max time of flight for the hub according to the turing data.
   */
  public static double getMaxHubTimeOfFlight() {
    return maxHubTimeOfFlight;
  }

  /**
   * @return The max time of flight for the hub according to the turing data.
   */
  public static double getMinHubTimeOfFlight() {
    return minHubTimeOfFlight;
  }

  public record TurretShotData(
      Distance distance, double RPM, double angleRad, double timeOfFlightSec) {}

  private static TurretShotData interpolate(TurretShotData start, TurretShotData end, double t) {
    return new TurretShotData(
        Meters.of(MathUtil.interpolate(start.distance.in(Meter), end.distance.in(Meter), t)),
        MathUtil.interpolate(start.RPM, end.RPM, t),
        MathUtil.interpolate(start.angleRad, end.angleRad, t),
        MathUtil.interpolate(start.timeOfFlightSec, end.timeOfFlightSec, t));
  }

  /** Load the real shot data */
  private static void loadRealValues() {

    hubDataPoints =
        new ArrayList<>(
            Arrays.asList(
                // new TurretShotData(Meters.of(2.01), 1600.0, 0.0, 0.53), // Out-dated
                // new TurretShotData(Meters.of(2.60), 1700.0, 0.0, 0.63),
                // new TurretShotData(Meters.of(2.17), 1650.0, 0.0, 0.42),
                // new TurretShotData(Meters.of(2.78), 1750.0, 0.0, 0.41),
                // new TurretShotData(Meters.of(3.4), 1900.0, 0.0, 0.63),
                new TurretShotData(Inches.of(76.5), 1640.0, 0.0, 0.861), // Time of flight verified
                new TurretShotData(Inches.of(88.5), 1700.0, 0.0, 0.911),
                new TurretShotData(Inches.of(100.5), 1750.0, 0.0, 0.931),
                new TurretShotData(Inches.of(112.5), 1810.0, 0.0, 0.989),
                new TurretShotData(Inches.of(124.5), 1880.0, 0.0, 1.045),
                new TurretShotData(Inches.of(133.5), 1920.0, 0.0, 1.054),
                new TurretShotData(
                    Meters.of(3.65), 1960.0, 0.0, 1.098), // predicted time of flight (recheck)
                new TurretShotData(Meters.of(4.05), 2000.0, 0.0, 1.153),
                new TurretShotData(Meters.of(4.36), 2100.0, 0.0, 1.196),
                new TurretShotData(Meters.of(4.46), 2150.0, 0.0, 1.210)));

    for (TurretShotData dataPoint : hubDataPoints) {
      hubMap.put(dataPoint.distance, dataPoint);
      velocityToEffectiveDistanceHubMap.put(
          dataPoint.distance.div(dataPoint.timeOfFlightSec).in(Meters),
          dataPoint.distance.in(Meters));

      if (dataPoint.timeOfFlightSec < minHubTimeOfFlight)
        minHubTimeOfFlight = dataPoint.timeOfFlightSec;
      if (dataPoint.timeOfFlightSec > maxHubTimeOfFlight)
        maxHubTimeOfFlight = dataPoint.timeOfFlightSec;
    }

    groundDataPoints =
        new ArrayList<>(
            Arrays.asList(
                new TurretShotData(Meters.of(5.0), 1700.0, 0.4, 1.0),
                new TurretShotData(Meters.of(6.0), 1800.0, 0.4, 1.0),
                new TurretShotData(Meters.of(7.0), 1900.0, 0.4, 1.0),
                new TurretShotData(Meters.of(8.0), 2000.0, 0.4, 1.0),
                new TurretShotData(Meters.of(9.0), 2100.0, 0.4, 1.0),
                new TurretShotData(Meters.of(10.0), 2500.0, 0.4, 1.0)));

    for (TurretShotData dataPoint : groundDataPoints) {
      groundMap.put(dataPoint.distance, dataPoint);
      velocityToEffectiveDistanceGroundMap.put(
          dataPoint.distance.div(dataPoint.timeOfFlightSec).in(Meters),
          dataPoint.distance.in(Meters));
    }
  }

  /** Load the sim shot data */
  private static void loadSimValues() {
    hubDataPoints =
        new ArrayList<>(
            Arrays.asList(
                new TurretShotData(Meters.of(2.54), 3050.0, 1.309, 0.0),
                new TurretShotData(Meters.of(5.08), 3100.0, 0.698, 0.0),
                new TurretShotData(Meters.of(5.17), 4100.0, 0.785, 0.0)));

    for (TurretShotData dataPoint : hubDataPoints) {
      hubMap.put(dataPoint.distance, dataPoint);
      velocityToEffectiveDistanceHubMap.put(
          dataPoint.distance.div(dataPoint.timeOfFlightSec).in(Meters),
          dataPoint.distance.in(Meters));

      if (dataPoint.timeOfFlightSec < minHubTimeOfFlight)
        minHubTimeOfFlight = dataPoint.timeOfFlightSec;
      if (dataPoint.timeOfFlightSec > maxHubTimeOfFlight)
        maxHubTimeOfFlight = dataPoint.timeOfFlightSec;
    }

    groundDataPoints =
        new ArrayList<>(Arrays.asList(new TurretShotData(Meters.of(0.0), 4000.0, 0.5, 0.0)));

    for (TurretShotData dataPoint : groundDataPoints) {
      groundMap.put(dataPoint.distance, dataPoint);
      velocityToEffectiveDistanceGroundMap.put(
          dataPoint.distance.div(dataPoint.timeOfFlightSec).in(Meters),
          dataPoint.distance.in(Meters));
    }
  }

  static {

    // set the real or sim data
    if (Constants.currentMode != Mode.SIM) {
      loadRealValues();
    } else {
      loadSimValues();
    }
  }

  /** Different target types for the turret to be tuned too. */
  public enum TargetType {
    /** The hub */
    HUB,

    /** The ground */
    GROUND,

    /** Used for dropping the turret flat */
    IN_TRENCH,

    /** Outside of the field */
    INVALID
  }
}
