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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class AutoDriveCommands {

  private static final PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
  private static final PPHolonomicDriveController preciseMoveController =
      new PPHolonomicDriveController(new PIDConstants(2.0, 0, 0), new PIDConstants(1.0, 0, 0));
  private static final Debouncer preciseMoveDebouncer = new Debouncer(1.0, DebounceType.kBoth);

  public static Command driveToPose(Drive drive, Pose2d pose, boolean withPreciseMove) {

    if (withPreciseMove) {
      return Commands.sequence(
              AutoBuilder.pathfindToPose(pose, constraints), preciseMove(drive, pose))
          .withName("driveToPoseWithPreciseMove");
    }

    return AutoBuilder.pathfindToPose(pose, constraints).withName("driveToPose");
  }

  public static Command driveToPoseThenPath(
      Drive drive, PathPlannerPath path, boolean withPreciseMove) {

    if (withPreciseMove) {
      return Commands.sequence(
              pathFindAndFollowPathFixer(path),
              preciseMove(
                  drive,
                  FieldConstants.correctSide(
                      path.getPathPoses().get(path.getPathPoses().size() - 1))))
          .withName("driveToPoseThenPathWithPreciseMove");
    }

    return pathFindAndFollowPathFixer(path).withName("driveToPoseThenPath");
  }

  private static Command pathFindAndFollowPathFixer(PathPlannerPath path) {
    return AutoBuilder.pathfindToPose(
            FieldConstants.correctSide(path.getPathPoses().get(0)), constraints)
        .andThen(AutoBuilder.followPath(path));
  }

  private static Command preciseMove(Drive drive, Pose2d pose) {

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
}
