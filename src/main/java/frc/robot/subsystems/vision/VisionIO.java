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
    MEGATAG_2,
    PHOTONVISION
  }

  /** Represents an object pose */
  public static record ObjectObservation(double timestamp, Pose2d pose, double certainty) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setWhiteList(int[] id) {}

  public default void clearWhiteList() {}

  public default void setIMUMode(int mode) {}
}
