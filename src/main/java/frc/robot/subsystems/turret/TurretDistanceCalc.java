// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meter;

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

  // The key must be in meters!!!
  private static InterpolatingTreeMap<Double, TurretShotData> hubMap =
      new InterpolatingTreeMap<Double, TurretShotData>(
          InverseInterpolator.forDouble(), TurretDistanceCalc::interpolate);
  private static InterpolatingTreeMap<Double, TurretShotData> groundMap =
      new InterpolatingTreeMap<Double, TurretShotData>(
          InverseInterpolator.forDouble(), TurretDistanceCalc::interpolate);

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
        return hubMap.get(distance.in(Meter));
      case GROUND:
        return groundMap.get(distance.in(Meter));
      case IN_TRENCH:
        // So that when the robot is under the trench the flywheel stays up to speed.
        TurretShotData unchangedValue = hubMap.get(distance.in(Meter));
        return new TurretShotData(
            unchangedValue.meters, unchangedValue.RPM, 0.0, unchangedValue.timeOfFlightSec);
      case INVALID:
        // Look at the hub to try to fix pose estimation
        return hubMap.get(distance.in(Meter));
    }

    return new TurretShotData(0, 0, 0, 0); // How?
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
      double meters, double RPM, double angleRad, double timeOfFlightSec) {}

  private static TurretShotData interpolate(TurretShotData start, TurretShotData end, double t) {
    return new TurretShotData(
        MathUtil.interpolate(start.meters, start.meters, t),
        MathUtil.interpolate(start.RPM, end.RPM, t),
        MathUtil.interpolate(start.angleRad, end.angleRad, t),
        MathUtil.interpolate(start.timeOfFlightSec, end.timeOfFlightSec, t));
  }

  /** Load the real shot data */
  private static void loadRealValues() {

    hubDataPoints =
        new ArrayList<>(
            Arrays.asList(
                new TurretShotData(2.17, 1650.0, 0.0, 0.42),
                new TurretShotData(2.78, 1750.0, 0.0, 0.41),
                new TurretShotData(3.4, 1850.0, 0.0, 0.63),
                new TurretShotData(3.65, 1900.0, 0.0, 0.57),
                new TurretShotData(4.36, 2100.0, 0.0, 1.0),
                new TurretShotData(4.46, 2200.0, 0.0, 1.08)));

    for (TurretShotData dataPoint : hubDataPoints) {
      hubMap.put(dataPoint.meters, dataPoint);
      velocityToEffectiveDistanceHubMap.put(
          dataPoint.meters / dataPoint.timeOfFlightSec, dataPoint.meters);

      if (dataPoint.timeOfFlightSec < minHubTimeOfFlight)
        minHubTimeOfFlight = dataPoint.timeOfFlightSec;
      if (dataPoint.timeOfFlightSec > maxHubTimeOfFlight)
        maxHubTimeOfFlight = dataPoint.timeOfFlightSec;
    }

    groundDataPoints = new ArrayList<>(Arrays.asList(new TurretShotData(0.0, 0.0, 0.0, 0.0)));

    for (TurretShotData dataPoint : groundDataPoints) {
      groundMap.put(dataPoint.meters, dataPoint);
      velocityToEffectiveDistanceGroundMap.put(
          dataPoint.meters / dataPoint.timeOfFlightSec, dataPoint.meters);
    }
  }

  /** Load the sim shot data */
  private static void loadSimValues() {
    hubDataPoints =
        new ArrayList<>(
            Arrays.asList(
                new TurretShotData(2.54, 3050.0, 1.309, 0.0),
                new TurretShotData(5.08, 3100.0, 0.698, 0.0),
                new TurretShotData(5.17, 4100.0, 0.785, 0.0)));

    for (TurretShotData dataPoint : hubDataPoints) {
      hubMap.put(dataPoint.meters, dataPoint);
      velocityToEffectiveDistanceHubMap.put(
          dataPoint.meters / dataPoint.timeOfFlightSec, dataPoint.meters);

      if (dataPoint.timeOfFlightSec < minHubTimeOfFlight)
        minHubTimeOfFlight = dataPoint.timeOfFlightSec;
      if (dataPoint.timeOfFlightSec > maxHubTimeOfFlight)
        maxHubTimeOfFlight = dataPoint.timeOfFlightSec;
    }

    groundDataPoints = new ArrayList<>(Arrays.asList(new TurretShotData(0.0, 4000.0, 0.5, 0.0)));

    for (TurretShotData dataPoint : groundDataPoints) {
      groundMap.put(dataPoint.meters, dataPoint);
      velocityToEffectiveDistanceGroundMap.put(
          dataPoint.meters / dataPoint.timeOfFlightSec, dataPoint.meters);
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
