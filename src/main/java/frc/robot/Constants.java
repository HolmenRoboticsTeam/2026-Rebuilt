// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Supplier<Boolean> isBlueAlliance =
      () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

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
                        181.56 - (47.0 / 2.0)
                        /** Minus one half hub behind the trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(316.64, Inches))));

    public static final Rectangle2d kLeftNeutralSide =
        new Rectangle2d(
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        181.56 + (47.0 / 2.0)
                        /** Plus one half hub in front of the trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(158.32, Inches))),
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        468.56 - (47.0 / 2.0)
                        /** Minus one half hub behind the opposing trench */
                        ,
                        Inches),
                    Distance.ofRelativeUnits(316.64, Inches))));

    public static final Rectangle2d kRightNeutralSide =
        new Rectangle2d(
            correctSide(
                new Translation2d(
                    Distance.ofRelativeUnits(
                        181.56 + (47.0 / 2.0)
                        /** Plus one half hub in front of the trench */
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

    // Auto Starting positions

    public static final Pose2d kLeftTrenchStart =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.600, Meter),
                Distance.ofRelativeUnits(7.400, Meter),
                Rotation2d.fromDegrees(0.0)));

    public static final Pose2d kHubStart =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.600, Meter),
                Distance.ofRelativeUnits(4.000, Meter),
                Rotation2d.fromDegrees(0.0)));

    public static final Pose2d kRightTrenchStart =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.600, Meter),
                Distance.ofRelativeUnits(0.600, Meter),
                Rotation2d.fromDegrees(0.0)));

    // Drive to Ball Launching positions

    public static final Pose2d kLeftDepotLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(1.500, Meter),
                Distance.ofRelativeUnits(6.000, Meter),
                Rotation2d.fromDegrees(0.0)));

    public static final Pose2d kLeftBumpLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.500, Meter),
                Distance.ofRelativeUnits(5.600, Meter),
                Rotation2d.fromDegrees(137.0)));

    public static final Pose2d kRightBumpLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(3.500, Meter),
                Distance.ofRelativeUnits(2.400, Meter),
                Rotation2d.fromDegrees(-137.0)));

    public static final Pose2d kRightOutpostLaunch =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(1.500, Meter),
                Distance.ofRelativeUnits(2.000, Meter),
                Rotation2d.fromDegrees(180.0)));

    // Climb Positions

    public static final Pose2d kLeftClimb =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(1.200, Meter),
                Distance.ofRelativeUnits(4.600, Meter),
                Rotation2d.fromDegrees(90.0)));

    public static final Pose2d kRightClimb =
        correctSide(
            new Pose2d(
                Distance.ofRelativeUnits(0.900, Meter),
                Distance.ofRelativeUnits(2.800, Meter),
                Rotation2d.fromDegrees(-90.0)));

    // #################### OTHER DATA ####################

    public static final int[] kHubTags =
        isBlueAlliance.get()
            ? new int[] {18, 19, 20, 21, 24, 25, 26, 27}
            : new int[] {2, 3, 4, 5, 8, 9, 10, 11};

    // #################### HELPER METHOD ####################

    public static Translation2d correctSide(Translation2d trans) {
      if (isBlueAlliance.get()) return trans;

      //   return new Translation2d(
      //       trans.getMeasureX().minus(fieldLength), trans.getMeasureY().minus(fieldWidth));
      return new Translation2d(
          fieldLength.minus(trans.getMeasureX()), fieldWidth.minus(trans.getMeasureY()));
    }

    public static Pose2d correctSide(Pose2d pose) {
      if (isBlueAlliance.get()) return pose;

      return new Pose2d(
          correctSide(pose.getTranslation()), pose.getRotation().minus(Rotation2d.k180deg));
    }
  }

  public static List<Pair<String, Command>> getNamedCommand(
      Drive drive,
      Vision vision,
      Intake intake,
      Indexer indexer,
      Feeder feeder,
      Turret turret,
      Climber climber) {
    return new ArrayList<Pair<String, Command>>(
        Arrays.asList(
            // Intake Commands
            new Pair<String, Command>("Intake Start", intake.start()),
            new Pair<String, Command>("Intake Stop", intake.stop()),

            // Indexer Commands
            new Pair<String, Command>("Indexer Auto", indexer.autoIndex()),
            new Pair<String, Command>(
                "Indexer Wait Until Out Of Fuel", Commands.waitUntil(() -> !indexer.hasFuel())),

            // Feeder Commands
            new Pair<String, Command>("Feeder Auto", feeder.autoFeed()),

            // Turret Commands
            new Pair<String, Command>("Turret Full Field Aim", turret.fullFieldAim()),

            // Climber Commands
            new Pair<String, Command>("Climber Calibrate", climber.calibrate()),
            new Pair<String, Command>("Climber Extend", climber.extend()),
            new Pair<String, Command>("Climber Retract", climber.retract())));
  }
}
