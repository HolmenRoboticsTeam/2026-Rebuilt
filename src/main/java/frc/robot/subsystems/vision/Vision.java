// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionPoseConsumer poseConsumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean ignoreVision;
  private VisionRotationConsumer rotationConsumer;

  // Initialize logging values
  List<Pose3d> allTagPoses = new LinkedList<>();
  List<Pose3d> allRobotPoses = new LinkedList<>();
  List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
  List<Pose3d> allRobotPosesRejected = new LinkedList<>();

  // Initialize logging values
  List<Pose3d> tagPoses = new LinkedList<>();
  List<Pose3d> robotPoses = new LinkedList<>();
  List<Pose3d> robotPosesAccepted = new LinkedList<>();
  List<Pose3d> robotPosesRejected = new LinkedList<>();

  public Vision(VisionPoseConsumer poseConsumer, VisionIO... io) {
    this.poseConsumer = poseConsumer;
    this.io = io;
    this.ignoreVision = false;
    this.rotationConsumer = (r) -> {};

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Pre load all field info to prevent freeze on first tag.
    for (int i = 1; i < 33; i++) {
      aprilTagLayout.getTagPose(i).get();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var poseObservation : inputs[cameraIndex].poseObservations) {

        // Check whether to reject pose
        boolean rejectPose =
            poseObservation.tagCount() == 0 // Must have at least one tag
                || (poseObservation.tagCount() == 1
                    && poseObservation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(poseObservation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || poseObservation.pose().getX() < 0.0
                || poseObservation.pose().getX() > aprilTagLayout.getFieldLength()
                || poseObservation.pose().getY() < 0.0
                || poseObservation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(poseObservation.pose());
        if (rejectPose) {
          robotPosesRejected.add(poseObservation.pose());
        } else {
          robotPosesAccepted.add(poseObservation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Skip if ignoring
        if (this.ignoreVision) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(poseObservation.averageTagDistance(), 2.0) / poseObservation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (poseObservation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        poseConsumer.acceptPose(
            poseObservation.pose().toPose2d(),
            poseObservation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  public Command setWhiteList(int[] ids) {
    return Commands.runOnce(
            () -> {
              for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
                io[cameraIndex].setWhiteList(ids);
              }
            },
            this)
        .withName("Vision_SetWhiteList");
  }

  public Command clearWhiteList() {
    return Commands.runOnce(
            () -> {
              for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
                io[cameraIndex].clearWhiteList();
              }
            },
            this)
        .withName("Vision_ClearWhiteList");
  }

  public Command setIMUMode(int mode) {
    return Commands.runOnce(
            () -> {
              for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
                io[cameraIndex].setIMUMode(mode);
              }
            },
            this)
        .ignoringDisable(true)
        .withName("Vision_SetIMUMode");
  }

  public Command ignoreVision(boolean ignoreVision) {
    return Commands.runOnce(
            () -> {
              this.ignoreVision = ignoreVision;
            })
        .ignoringDisable(true)
        .withName("Vision_ignoreVision");
  }

  public void setRotationConsumer(VisionRotationConsumer consumer) {
    this.rotationConsumer = consumer;
  }

  public static interface VisionPoseConsumer {
    public void acceptPose(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public static interface VisionRotationConsumer {
    public void acceptRotation(Rotation2d visionRobotRotation);
  }
}
