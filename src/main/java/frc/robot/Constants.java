// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FieldConstants {

    public static final Distance fieldLength = Inches.of(650.12);
    public static final Distance fieldWidth = Inches.of(316.64);

    // #################### RECTANGLE ZONES ####################

    public static final Rectangle2d kWholeField =
        new Rectangle2d(
            new Translation2d(Inches.of(0.0), Inches.of(0.0)),
            new Translation2d(fieldLength, fieldWidth));

    public static final Rectangle2d kHomeAllianceZone =
        new Rectangle2d( // Blue Alliance Zone
            new Translation2d(Inches.of(0.0), Inches.of(0.0)),
            new Translation2d(Inches.of(157.06), fieldWidth));

    public static final Rectangle2d kAllianceTrenchBumpZone =
        new Rectangle2d(
            new Translation2d(Inches.of(157.06), Inches.of(0.0)),
            new Translation2d(Inches.of(206.06), fieldWidth));

    public static final Rectangle2d kLeftNeutralSide =
        new Rectangle2d(
            new Translation2d(Inches.of(206.06), Inches.of(158.32)),
            new Translation2d(Inches.of(445.06), fieldWidth));

    public static final Rectangle2d kRightNeutralSide =
        new Rectangle2d(
            new Translation2d(Inches.of(206.06), Inches.of(0.0)),
            new Translation2d(Inches.of(445.06), Inches.of(158.32)));

    public static final Rectangle2d kOpposingTrenchBumpZone =
        new Rectangle2d(
            new Translation2d(Inches.of(445.06), Inches.of(0.0)),
            new Translation2d(Inches.of(492.06), fieldWidth));

    public static final Rectangle2d kLeftOpposingSide =
        new Rectangle2d(
            new Translation2d(Inches.of(492.06), Inches.of(158.32)),
            new Translation2d(fieldLength, fieldWidth));

    public static final Rectangle2d kRightOpposingSide =
        new Rectangle2d(
            new Translation2d(Inches.of(492.06), Inches.of(0.0)),
            new Translation2d(fieldLength, Inches.of(158.32)));

    // #################### TRANSLATIONS POINTS ####################

    public static final Translation2d kHubPosition =
        new Translation2d(Inches.of(181.56), Inches.of(158.32));

    public static final Translation2d kLeftCorner =
        new Translation2d(
            Inches.of(15.50), fieldWidth.minus(Inches.of(100.0))
            /** Left Wall Minus Inches */
            );

    public static final Translation2d kRightCorner =
        new Translation2d(Inches.of(15.50), Inches.of(100.0));

    // #################### POSE POSITIONS ####################

    // Auto Starting positions

    public static final Pose2d kLeftTrenchStart =
        new Pose2d(Meters.of(3.600), Meters.of(7.400), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kHubStart =
        new Pose2d(Meters.of(3.600), Meters.of(4.000), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kRightTrenchStart =
        new Pose2d(Meters.of(3.600), Meters.of(0.600), Rotation2d.fromDegrees(0.0));

    // Drive to Ball Launching positions

    public static final Pose2d kLeftDepotLaunch =
        new Pose2d(Meters.of(1.500), Meters.of(6.000), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLeftBumpLaunch =
        new Pose2d(Meters.of(3.500), Meters.of(5.600), Rotation2d.fromDegrees(137.0));

    public static final Pose2d kRightBumpLaunch =
        new Pose2d(Meters.of(3.500), Meters.of(2.400), Rotation2d.fromDegrees(-137.0));

    public static final Pose2d kRightOutpostLaunch =
        new Pose2d(Meters.of(1.500), Meters.of(2.000), Rotation2d.fromDegrees(180.0));

    // Climb Positions

    public static final Pose2d kLeftClimb =
        new Pose2d(Meters.of(1.200), Meters.of(4.600), Rotation2d.fromDegrees(90.0));

    public static final Pose2d kRightClimb =
        new Pose2d(Meters.of(0.900), Meters.of(2.800), Rotation2d.fromDegrees(-90.0));

    // #################### OTHER DATA ####################

    public static final int[] kHubTags =
        isBlueAlliance.get()
            ? new int[] {18, 19, 20, 21, 24, 25, 26, 27} // Blue hub
            : new int[] {2, 3, 4, 5, 8, 9, 10, 11}; // Red hub

    // #################### HELPER METHOD ####################

    public static Translation2d allianceFlip(Translation2d trans) {
      if (isBlueAlliance.get()) return trans;

      return new Translation2d(
          fieldLength.minus(trans.getMeasureX()), fieldWidth.minus(trans.getMeasureY()));
    }

    public static Pose2d allianceFlip(Pose2d pose) {
      if (isBlueAlliance.get()) return pose;

      return new Pose2d(
          allianceFlip(pose.getTranslation()), pose.getRotation().minus(Rotation2d.k180deg));
    }

    public static Rectangle2d allianceFlip(Rectangle2d rect) {
      if (isBlueAlliance.get()) return rect;

      return new Rectangle2d(allianceFlip(rect.getCenter()), rect.getXWidth(), rect.getYWidth());
    }
  }

  public static List<Pair<String, Command>> getNamedCommand(
      Drive drive, Vision vision, Intake intake, Indexer indexer, Feeder feeder, Turret turret) {
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
            new Pair<String, Command>(
                "Feeder Auto",
                feeder
                    .autoFeed()
                    .alongWith(
                        Commands.sequence(
                            intake.reverse(), Commands.waitSeconds(3.0), intake.start()))),

            // Turret Commands
            new Pair<String, Command>("Turret Full Field Aim", turret.fullFieldAim())));
  }
}
