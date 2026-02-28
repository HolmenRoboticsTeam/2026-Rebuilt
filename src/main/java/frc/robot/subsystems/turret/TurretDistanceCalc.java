// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

/** Add your docs here. */
public class TurretDistanceCalc {

  // The key must be in inches!!!
  private static InterpolatingTreeMap<Double, TurretShotData> hubMap =
      new InterpolatingTreeMap<Double, TurretShotData>(
          InverseInterpolator.forDouble(), TurretDistanceCalc::interpolate);
  private static InterpolatingTreeMap<Double, TurretShotData> groundMap =
      new InterpolatingTreeMap<Double, TurretShotData>(
          InverseInterpolator.forDouble(), TurretDistanceCalc::interpolate);

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
        return hubMap.get(distance.in(Inches));
      case GROUND:
        return groundMap.get(distance.in(Inches));
      case IN_TRENCH:
        // So that when the robot is under the trench the flywheel stays up to speed.
        TurretShotData unchangedValue = hubMap.get(distance.in(Inches));
        return new TurretShotData(unchangedValue.RPM, 0.0, unchangedValue.timeOfFlightSec);
      case INVALID:
        // Look at the hub to try to fix pose estimation
        return hubMap.get(distance.in(Inches));
    }

    return new TurretShotData(0, 0, 0); // How?
  }

  private static void loadRealValues() {
    // TODO: Add real values
    hubMap.put(25.0, new TurretShotData(0, 0, 0.0));
    hubMap.put(50.0, new TurretShotData(0, 0.1, 0.0));
    hubMap.put(75.0, new TurretShotData(0, 0.2, 0.0));
    hubMap.put(100.0, new TurretShotData(0, 0.3, 0.0));
    hubMap.put(125.0, new TurretShotData(0, 0.4, 0.0));
    hubMap.put(150.0, new TurretShotData(0, 0.5, 0.0));
  }

  private static void loadSimValues() {
    hubMap.put(100.0, new TurretShotData(3050.0, 1.309, 0.0));
    hubMap.put(200.0, new TurretShotData(3100.0, 0.698, 0.0));
    hubMap.put(225.0, new TurretShotData(4100.0, 0.785, 0.0));
  }

  static {

    // Load some defaults
    groundMap.put(0.0, new TurretShotData(0, 0, 0.0));

    // set the real or sim data
    if (Constants.currentMode == Mode.REAL) {
      loadRealValues();
    } else {
      loadSimValues();
    }
  }

  public static double getMaxTimeOfFlight() {
    return 2.5;
  }

  public static double getMinTimeOfFlight() {
    return 1.0;
  }

  public record TurretShotData(double RPM, double angleRad, double timeOfFlightSec) {}

  public static TurretShotData interpolate(TurretShotData start, TurretShotData end, double t) {
    return new TurretShotData(
        MathUtil.interpolate(start.RPM, end.RPM, t),
        MathUtil.interpolate(start.angleRad, end.angleRad, t),
        MathUtil.interpolate(start.timeOfFlightSec, end.timeOfFlightSec, t));
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
