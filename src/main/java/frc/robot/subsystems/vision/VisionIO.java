// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this projectR

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public ObjectObservation[] objectObservations = new ObjectObservation[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /** Represents an object pose */
  public static record ObjectObservation(double timestamp, Pose2d pose, double certainty) {}

  /**
   * Updates the inputs for the implementation of the vision.
   *
   * @param inputs The class of inputs to update.
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Sets which tags the camera is allowed to pull estimations from.
   *
   * @param id which tags
   */
  public default void setWhiteList(int[] id) {}

  /** Clears the which tags the camera is allowed to pull estimations from. */
  public default void clearWhiteList() {}

  /**
   * Sets the IUM mode of the limelights
   *
   * @param mode the target mode
   */
  public default void setIMUMode(int mode) {}

  /**
   * Records the last amount of seconds that the limelight saw.
   *
   * @param seconds the amount of seconds
   */
  public default void recordLastSeconds(double seconds) {}
}
