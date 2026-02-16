// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean isBlueAlliance =
      DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FieldConstants {

    public static final Distance fieldLength = Distance.ofRelativeUnits(651.22, Inches);
    public static final Distance fieldWidth = Distance.ofRelativeUnits(317.69, Inches);

    // #################### RECTANGLE ZONES ####################

    public static final Rectangle2d kHomeAllianceZone =
        new Rectangle2d( // Blue Alliance Zone
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(0.0, Inches), Distance.ofRelativeUnits(0.0, Inches))),
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        181.56 - 47.0
                        /** Minus one full hub behind the trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(316.64, Inches))));

    public static final Rectangle2d kLeftNeutralSide =
        new Rectangle2d(
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        181.56 + 47.0
                        /** Plus one full hub in front of the trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(158.32, Inches))),
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        468.56 - 47.0
                        /** Minus one full hub behind the opposing trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(316.64, Inches))));

    public static final Rectangle2d kRightNeutralSide =
        new Rectangle2d(
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        181.56 + 47.0
                        /** Plus one full hub in front of the trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(0.0, Inches))),
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        468.56 - 47.0
                        /** Minus one full hub behind the opposing trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(158.8, Inches))));

    // #################### TRANSLATIONS POINTS ####################

    public static final Translation2d kHubPosition =
        correctSide(
            new Translation2d(
                Distance.ofRelativeUnits(181.56, Inches),
                Distance.ofRelativeUnits(158.32, Inches)));

    public static final Translation2d kLeftCorner =
        correctSide(
            new Translation2d(
                Distance.ofRelativeUnits(15.50, Inches),
                Distance.ofRelativeUnits(
                    316.64 - 75.0
                    /** Left Wall Minus Inches */
                    ,
                    Inches)));

    public static final Translation2d kRightCorner =
        correctSide(
            new Translation2d(
                Distance.ofRelativeUnits(15.50, Inches), Distance.ofRelativeUnits(75.0, Inches)));

    // #################### POSE POSITIONS ####################

    public static final Pose2d kLeftTrenchLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.500, Meter),
                Distance.ofRelativeUnits(7.400, Meter),
                Rotation2d.fromDegrees(-90.0)));

    public static final Pose2d kRightTrenchLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.500, Meter),
                Distance.ofRelativeUnits(0.6, Meter),
                Rotation2d.fromDegrees(90.0)));

    public static final Pose2d kLeftClimb =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(0.800, Meter),
                Distance.ofRelativeUnits(4.600, Meter),
                Rotation2d.fromDegrees(0.0)));

    public static final Pose2d kRightClimb =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(1.400, Meter),
                Distance.ofRelativeUnits(2.800, Meter),
                Rotation2d.fromDegrees(180.0)));

    // #################### OTHER DATA ####################

    public static final int[] kHubTags =
        isBlueAlliance
            ? new int[] {18, 19, 20, 21, 24, 25, 26, 27}
            : new int[] {2, 3, 4, 5, 8, 9, 10, 11};

    // #################### HELPER METHOD ####################

    private static Translation2d correctSide(Translation2d trans) {
      if (isBlueAlliance) return trans;

      return new Translation2d(
          trans.getMeasureX().minus(fieldLength), trans.getMeasureY().minus(fieldWidth));
    }

    private static Pose2d correctSide(Pose2d pose) {
      if (isBlueAlliance) return pose;

      return new Pose2d(
          correctSide(pose.getTranslation()), pose.getRotation().minus(Rotation2d.k180deg));
    }
  }
}
