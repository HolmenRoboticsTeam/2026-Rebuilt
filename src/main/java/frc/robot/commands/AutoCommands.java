// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

/** Add your docs here. */
public class AutoCommands {

  public static Command displayAutoField(
      Field2d field, Supplier<String> autoName, Supplier<Pose2d> robotPose) {
    double[] robotIndex = new double[] {0};
    return Commands.sequence(
            Commands.run(
                    () -> {

                      // Get auto poses
                      List<Pose2d> autoPoses = getAutoPoses(autoName);

                      // Save trajectories.
                      field.getObject("traj").setPoses(autoPoses);

                      robotIndex[0] += 1.0 / 3.0;
                      if (robotIndex[0] > autoPoses.size() - 2) {
                        robotIndex[0] = 0.0;
                      }
                      int index = (int) Math.floor(robotIndex[0]);

                      field.setRobotPose(autoPoses.get(index));
                    })
                .until(() -> DriverStation.isEnabled()),
            Commands.run(
                    () -> {
                      field.setRobotPose(robotPose.get());
                    })
                .until(() -> DriverStation.isDisabled()))
        .ignoringDisable(true)
        .withName("displayAutoField");
  }

  private static List<Pose2d> getAutoPoses(Supplier<String> autoName) {
    // Initialize a list of poses
    List<Pose2d> autoPoses = new ArrayList<>();

    // Get the poses from paths, and add them to the list of all poses.
    try {
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName.get());
      for (PathPlannerPath path : paths) {

        // Flip if on red alliance
        if (!Constants.isBlueAlliance.get()) path = path.flipPath();

        // Add poses from each path
        autoPoses.addAll(path.getPathPoses());
      }
    } catch (IOException | ParseException e) {
      return new ArrayList<>();
    }

    // Add poses to a set, and remove duplicates
    Set<Pose2d> set = new LinkedHashSet<>();
    for (int i = 0; i < autoPoses.size(); i++) {
      if (set.contains(autoPoses.get(i))) {
        autoPoses.remove(i);
        i--;
      } else {
        set.add(autoPoses.get(i));
      }
    }

    // Trajectories don't show when their less than 9 poses, so add up to nine poses.
    while (autoPoses.size() < 9) autoPoses.add(autoPoses.get(autoPoses.size() - 1));

    // Pathplanner's paths do not come with rotations in their poses, so calculate them.
    for (int index = 0; index < autoPoses.size() - 1; index++) {
      // Get the difference in the next and the current translation.
      Translation2d deltaTranslation =
          autoPoses.get(index + 1).getTranslation().minus(autoPoses.get(index).getTranslation());

      // If they are on top of each other
      Rotation2d rot =
          deltaTranslation.equals(new Translation2d())
              ? new Rotation2d()
              : deltaTranslation.getAngle();

      autoPoses.set(index, new Pose2d(autoPoses.get(index).getTranslation(), rot));
    }

    return autoPoses;
  }
}
