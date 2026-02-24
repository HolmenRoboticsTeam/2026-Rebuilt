// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretDistanceCalc.TargetType;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class AutoCommandBuilder {

  private LoggedDashboardChooser<StartingPose> startingPose =
      new LoggedDashboardChooser<>("AutoBuilder/StartingPose");
  private LoggedDashboardChooser<FuelZone> firstFuelZone =
      new LoggedDashboardChooser<>("AutoBuilder/FirstFuelZone");
  private LoggedDashboardChooser<FuelZone> secondFuelZone =
      new LoggedDashboardChooser<>("AutoBuilder/SecondFuelZone");
  private LoggedDashboardChooser<ClimbSide> climbSide =
      new LoggedDashboardChooser<>("AutoBuilder/ClimbSide");

  private Drive drive;
  private Vision vision;
  private Intake intake;
  private Hopper hopper;
  private Indexer indexer;
  private Feeder feeder;
  private Turret turret;
  private Climber climber;

  public AutoCommandBuilder(
      Drive drive,
      Vision vision,
      Intake intake,
      Hopper hopper,
      Indexer indexer,
      Feeder feeder,
      Turret turret,
      Climber climber) {
    this.drive = drive;
    this.vision = vision;
    this.intake = intake;
    this.hopper = hopper;
    this.indexer = indexer;
    this.feeder = feeder;
    this.turret = turret;
    this.climber = climber;

    startingPose.addDefaultOption("PLEASE PICK ONE!", null);
    startingPose.addOption("Left Trench", StartingPose.LEFT_TRENCH);
    startingPose.addOption("Hub", StartingPose.HUB);
    startingPose.addOption("Right Trench", StartingPose.RIGHT_TRENCH);

    firstFuelZone.addDefaultOption("Skip", null);
    firstFuelZone.addOption("Collect Left Fuel", FuelZone.LEFT);
    firstFuelZone.addOption("Collect Left Neutral Zone Fuel", FuelZone.LEFT_NEUTRAL_ZONE_SHORT);
    firstFuelZone.addOption("Collect Right Neutral Zone Fuel", FuelZone.RIGHT_NEUTRAL_ZONE_SHORT);
    firstFuelZone.addOption("Collect Right Fuel", FuelZone.RIGHT);

    secondFuelZone.addDefaultOption("Skip", null);
    secondFuelZone.addOption("Collect Left Fuel", FuelZone.LEFT);
    secondFuelZone.addOption("Collect Left Neutral Zone Fuel", FuelZone.LEFT_NEUTRAL_ZONE_SHORT);
    secondFuelZone.addOption("Collect Right Neutral Zone Fuel", FuelZone.RIGHT_NEUTRAL_ZONE_SHORT);
    secondFuelZone.addOption("Collect Right Fuel", FuelZone.RIGHT);

    climbSide.addDefaultOption("PLEASE PICK ONE!", null);
    climbSide.addOption("Climb Left", ClimbSide.LEFT_CLIMB);
    climbSide.addOption("Skip Climb", ClimbSide.SKIP_CLIMB);
    climbSide.addOption("Climb Right", ClimbSide.RIGHT_CLIMB);
  }

  /**
   * Builds the auto command based on the DashboardChooser.
   *
   * @return The auto command
   * @throws FileVersionException
   * @throws IOException
   * @throws ParseException
   */
  public Command buildAutoCommand() throws FileVersionException, IOException, ParseException {

    StartingPose startingPose = this.startingPose.get();
    FuelZone firstFuelZone = this.firstFuelZone.get();
    FuelZone secondFuelZone = this.secondFuelZone.get();
    ClimbSide climbSide = this.climbSide.get();

    // If first fuel zone was skipped, but second was not, then swap the two.
    // This ensures that if a fuel zone is skipped, it must be the second one.
    if (firstFuelZone == null && secondFuelZone != null) {
      firstFuelZone = secondFuelZone;
      secondFuelZone = null;
    }

    // If there is no second fuel zone, upgrade the first path so more fuel is grabbed.
    // This ensures that if only one cycle of fuels is being grabbed, make it a big grab.
    if (secondFuelZone == null) {
      firstFuelZone = firstFuelZone.upgrade();
    }

    // If both first and second fuel zones are the same, upgrade the second one.
    // This ensures that new fuel is always being grabbed on each cycle.
    if (firstFuelZone == secondFuelZone) {
      secondFuelZone = secondFuelZone.upgrade();
    }

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startingPose.pose)),
        Commands.sequence(

                // First cycle
                collectFuel(firstFuelZone),
                shootFuel(firstFuelZone),

                // Second cycle
                collectFuel(secondFuelZone),
                shootFuel(secondFuelZone),

                // Keep shooting until it is time to climb
                Commands.parallel(
                    turret.fullFieldAim(),
                    Commands.repeatingSequence(
                        feeder.stop(),
                        Commands.waitUntil(() -> turret.isReadyForFuel()),
                        feeder.start(),
                        Commands.waitUntil(() -> !turret.isReadyForFuel()))))
            // Calibrate the climber
            .alongWith(climber.calibrate())
            // The robot is climbing and little time is left.
            .until(() -> climbSide != ClimbSide.SKIP_CLIMB && Timer.getMatchTime() < 7.0),

        /** Stage 3: Climb Extend climber Move to climb (While shooting) Retract climber */
        climb(climbSide));
  }

  /**
   * Creates and returns a command that drives to the target fuel zone, while intaking.
   *
   * @param fuelZone the target fuel zone
   * @return A command with the given logic
   * @throws FileVersionException
   * @throws IOException
   * @throws ParseException
   */
  public Command collectFuel(FuelZone fuelZone)
      throws FileVersionException, IOException, ParseException {
    return Commands.sequence(

            // Set the subsystems
            intake.start(),
            hopper.start(),
            indexer.start(),
            feeder.stop(),
            // Turret is running RPM and pointing at hub, but 0 angle (see deadline).

            // Drive the target path
            AutoDriveCommands.driveToPoseThenPath(
                drive, PathPlannerPath.fromPathFile(fuelZone.fuelPathName), false))
        .deadlineFor(turret.forceAim(FieldConstants.kHubPosition, TargetType.INVALID));
  }

  /**
   * Creates and returns a command that drives to the target shooting pose (based on fuel zone),
   * over the bump if needed. Then, starts feeding the turret when the turret is ready.
   *
   * @param fuelZone the associated fuel zone
   * @return A command with the given logic
   * @throws FileVersionException
   * @throws IOException
   * @throws ParseException
   */
  public Command shootFuel(FuelZone fuelZone)
      throws FileVersionException, IOException, ParseException {
    return Commands.sequence(

            // Set the subsystems
            intake.start(),
            hopper.start(),
            indexer.start(),
            feeder.stop(),
            // Turret is running fullFieldAim (see deadline).

            // Get back to the launch position
            // If the robot is in the neutral zone, it much drive over the bump.
            fuelZone.launchPathName == null
                ? AutoDriveCommands.driveToPose(drive, fuelZone.launchPose, false)
                : AutoDriveCommands.driveToPoseThenPath(
                    drive, PathPlannerPath.fromPathFile(fuelZone.launchPathName), false),

            // Start feeding when turret is ready
            Commands.repeatingSequence(
                    feeder.stop(),
                    Commands.waitUntil(() -> turret.isReadyForFuel()),
                    feeder.start(),
                    Commands.waitUntil(() -> !turret.isReadyForFuel()))
                .withTimeout(5.0) // This timeout is for when the robot is out of fuel.
            )
        .deadlineFor(turret.fullFieldAim());
  }

  /**
   * Creates and returns a command that extends the climber, alines to the selected climb side, and
   * climbs (the robot is allow to shoot while moving).
   *
   * @param climbSide the target climb side
   * @return A command with the given logic
   * @throws FileVersionException
   * @throws IOException
   * @throws ParseException
   */
  public Command climb(ClimbSide climbSide)
      throws FileVersionException, IOException, ParseException {
    return Commands.sequence(
            // Set the subsystems
            intake.start(),
            hopper.start(),
            indexer.start(),
            // Feeder and Turret are shooting still (see deadline).

            climber.extend(),

            // Drive over the bump, if needed
            AutoDriveCommands.driveToPoseThenPath(
                    drive, PathPlannerPath.fromPathFile(climbSide.bumpPathName), false)
                .onlyIf(
                    () ->
                        !FieldConstants.kHomeAllianceZone.contains(
                            drive.getPose().getTranslation())),

            // Drive to the selected climb side
            AutoDriveCommands.driveToPoseThenPath(
                drive, PathPlannerPath.fromPathFile(climbSide.climbPathName), false),

            // Lit the robot off the ground (hopefully!)
            climber.retract(),

            // Continue alining, while climbing
            AutoDriveCommands.driveToPose(drive, climbSide.pose, true).repeatedly())
        .deadlineFor(
            // Simple aim and shoot
            turret.fullFieldAim(),
            Commands.repeatingSequence(
                    feeder.stop(),
                    Commands.waitUntil(() -> turret.isReadyForFuel()),
                    feeder.start(),
                    Commands.waitUntil(() -> !turret.isReadyForFuel()))
                // Don't start shooting until the robot is in the alliance zone
                .beforeStarting(
                    Commands.waitUntil(
                        () ->
                            FieldConstants.kHomeAllianceZone.contains(
                                drive.getPose().getTranslation()))));
  }

  private final Supplier<PathPlannerPath> blankPath =
      () ->
          PathPlannerPath.fromPathPoints(
              new LinkedList<PathPoint>(
                  Arrays.asList(new PathPoint(drive.getPose().getTranslation()))),
              new PathConstraints(1.0, 1.0, 1.0, 1.0),
              new GoalEndState(0.0, drive.getRotation()));

  private enum StartingPose {
    LEFT_TRENCH(FieldConstants.kLeftTrenchStart),

    HUB(FieldConstants.kHubStart),

    RIGHT_TRENCH(FieldConstants.kRightTrenchStart);

    private final Pose2d pose;

    StartingPose(Pose2d pose) {
      this.pose = pose;
    }
  }

  private enum FuelZone {
    LEFT("Collect Depot", null, FieldConstants.kLeftDepotLaunch) {
      @Override // This does nothing
      public FuelZone upgrade() {
        return LEFT;
      }
    },

    LEFT_NEUTRAL_ZONE_SHORT(
        "Collect Left Neutral Zone Short",
        "Over Left Bump To Home",
        FieldConstants.kLeftBumpLaunch) {
      @Override // Changes to use LONG
      public FuelZone upgrade() {
        return LEFT_NEUTRAL_ZONE_LONG;
      }
    },

    LEFT_NEUTRAL_ZONE_LONG(
        "Collect Left Neutral Zone Long",
        "Over Left Bump To Home",
        FieldConstants.kLeftBumpLaunch) {
      @Override // This does nothing
      public FuelZone upgrade() {
        return LEFT_NEUTRAL_ZONE_LONG;
      }
    },

    RIGHT_NEUTRAL_ZONE_LONG(
        "Collect Right Neutral Zone Long",
        "Over Right Bump To Home",
        FieldConstants.kRightBumpLaunch) {
      @Override // This does nothing
      public FuelZone upgrade() {
        return RIGHT_NEUTRAL_ZONE_LONG;
      }
    },

    RIGHT_NEUTRAL_ZONE_SHORT(
        "Collect Right Neutral Zone Short",
        "Over Right Bump To Home",
        FieldConstants.kRightBumpLaunch) {
      @Override // Changes to use LONG
      public FuelZone upgrade() {
        return RIGHT_NEUTRAL_ZONE_LONG;
      }
    },

    RIGHT("Collect Outpost", null, FieldConstants.kRightOutpostLaunch) {
      @Override // This does nothing
      public FuelZone upgrade() {
        return RIGHT;
      }
    };

    private final String fuelPathName;
    private final String launchPathName;
    private final Pose2d launchPose;

    FuelZone(String fuelPathName, String launchPathName, Pose2d launchPose) {
      this.fuelPathName = fuelPathName;
      this.launchPathName = launchPathName;
      this.launchPose = launchPose;
    }

    public abstract FuelZone upgrade();
  }

  private enum ClimbSide {
    LEFT_CLIMB("Left Climb", "Over Left Bump To Home", FieldConstants.kLeftClimb),

    SKIP_CLIMB(null, null, null),

    RIGHT_CLIMB("Right Climb", "Over Right Bump To Home", FieldConstants.kRightClimb);

    private final String climbPathName;
    private final String bumpPathName;
    private final Pose2d pose;

    ClimbSide(String pathName, String bumpPathName, Pose2d pose) {
      this.climbPathName = pathName;
      this.bumpPathName = bumpPathName;
      this.pose = pose;
    }
  }
}
