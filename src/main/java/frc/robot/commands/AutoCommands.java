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
import java.util.List;
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
                      List<Pose2d> autoPoses = new ArrayList<>();

                      try {
                        for (PathPlannerPath path :
                            PathPlannerAuto.getPathGroupFromAutoFile(autoName.get())) {

                          if (!Constants.isBlueAlliance.get()) path = path.flipPath();

                          autoPoses.addAll(path.getPathPoses());
                        }
                      } catch (IOException | ParseException e) {
                        return;
                      }

                      field.getObject("traj").setPoses(autoPoses);
                      robotIndex[0] += 1.0 / 3.0;
                      if (robotIndex[0] > autoPoses.size() - 2) {
                        robotIndex[0] = 0.0;
                      }

                      int index = (int) Math.floor(robotIndex[0]);
                      Translation2d deltaTranslation =
                          autoPoses
                              .get(index + 1)
                              .getTranslation()
                              .minus(autoPoses.get(index).getTranslation());

                      Rotation2d rot =
                          deltaTranslation.equals(new Translation2d())
                              ? new Rotation2d()
                              : deltaTranslation.getAngle();

                      Pose2d pose = new Pose2d(autoPoses.get(index).getTranslation(), rot);

                      field.setRobotPose(pose);
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
}
