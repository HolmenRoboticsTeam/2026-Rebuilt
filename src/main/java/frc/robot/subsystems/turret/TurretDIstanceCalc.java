// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class TurretDIstanceCalc {
  private static InterpolatingDoubleTreeMap hubRPMMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap hubAngleMap = new InterpolatingDoubleTreeMap();

  static {
    hubRPMMap.put(2.0, 4000.0);
    hubAngleMap.put(2.0, Math.toRadians(45));
  }

  public static double getRPM(double meters) {
    return hubRPMMap.get(meters);
  }

  public static double getAngle(double meters) {
    return hubAngleMap.get(meters);
  }
}
