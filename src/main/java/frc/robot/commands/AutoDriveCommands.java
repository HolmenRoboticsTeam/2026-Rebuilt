// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import org.json.simple.parser.ParseException;

/**
 * A class for commands that control the robot's drive base through pathplanner's pathfinding,
 * follow paths, and a new precise move system.
 */
public class AutoDriveCommands {

  private static final PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
  private static final PPHolonomicDriveController preciseMoveController =
      new PPHolonomicDriveController(new PIDConstants(2.0, 0, 0), new PIDConstants(1.0, 0, 0));
  private static final Debouncer preciseMoveDebouncer = new Debouncer(1.0, DebounceType.kBoth);

  /**
   * Creates and returns a command that drives the robot to the given pose, using Pathplanner's
   * pathfinding. With the option to precisely move to the target, should it be needed.
   *
   * @param drive the drive subsystem
   * @param pose the target pose
   * @param withPreciseMove whether or not to move precisely to the target
   * @return A command with the given logic.
   */
  public static Command driveToPose(Drive drive, Pose2d pose, boolean withPreciseMove) {

    if (withPreciseMove) {
      return Commands.sequence(
              AutoBuilder.pathfindToPose(pose, constraints), preciseMove(drive, pose))
          .withName("driveToPoseWithPreciseMove");
    }

    return AutoBuilder.pathfindToPose(pose, constraints).withName("driveToPose");
  }

  /**
   * Creates and returns a command that drives the robot to the given starting pose of the given
   * path, using Pathplanner's pathfinding, then follows that path. With the option to precisely
   * move to the target, should it be needed.
   *
   * @param drive the drive subsystem
   * @param pathName the path to follow
   * @param withPreciseMove whether or not to move precisely to the target
   * @return A command with the given logic.
   */
  public static Command driveToPoseThenPath(Drive drive, String pathName, boolean withPreciseMove) {

    PathPlannerPath path = getPath(pathName);

    if (withPreciseMove) {
      return Commands.sequence(
              pathFindAndFollowPathFixer(path),
              preciseMove(drive, path.getPathPoses().get(path.getPathPoses().size() - 1)))
          .withName("driveToPoseThenPathWithPreciseMove");
    }

    return pathFindAndFollowPathFixer(path).withName("driveToPoseThenPath");
  }

  /**
   * Fixes an issue with paths not auto starting went using Pathplanner's default "Pathfind Then
   * Follow Path".
   *
   * @param path The path to fix
   * @return A fixed command.
   */
  private static Command pathFindAndFollowPathFixer(PathPlannerPath path) {

    Pose2d startOfPathPose =
        new Pose2d(
            path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation());
    return Commands.either(
            AutoBuilder.pathfindToPose(
                startOfPathPose, constraints, path.getIdealStartingState().velocityMPS()),
            AutoBuilder.pathfindToPose(
                FieldConstants.forceAllianceFlip(startOfPathPose),
                constraints,
                path.getIdealStartingState().velocityMPS()),
            () -> Constants.isBlueAlliance.get())
        .andThen(AutoBuilder.followPath(path));
  }

  /**
   * Precisely moves to the given pose. Note: This will drive the robot at the target pose, ignoring
   * obstacles.
   *
   * @param drive the drive subsystem
   * @param pose the target pose
   * @return A command with the given logic.
   */
  private static Command preciseMove(Drive drive, Pose2d pose) {

    // A supplier for if the robot is in position
    BooleanSupplier inPosition =
        () ->
            drive.getPose().getTranslation().getDistance(pose.getTranslation()) < 0.05 // Meters
                && Math.abs(drive.getPose().getRotation().minus(pose.getRotation()).getDegrees())
                    < 1.0; // Degrees

    return Commands.run(
            () -> {
              PathPlannerTrajectoryState goalPose = new PathPlannerTrajectoryState();
              goalPose.pose = pose;

              drive.runVelocity(
                  preciseMoveController.calculateRobotRelativeSpeeds(drive.getPose(), goalPose));
            })
        .until(() -> preciseMoveDebouncer.calculate(inPosition.getAsBoolean()));
  }

  /**
   * Builds and returns a pathplanner path given its name. Null if any IO exception is encountered.
   *
   * @param pathName the path name
   * @return the Pathplanner path
   */
  private static PathPlannerPath getPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (FileVersionException | IOException | ParseException e) {
      System.out.println(
          "PathPlannerIOException was caught in AutoDriveCommands. Name: " + pathName);
      return null;
    }
  }
}
