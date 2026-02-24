// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final Navx navX =
      new Navx(DriveConstants.navXCanId, (int) DriveConstants.odometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(() -> navX.getYaw().in(Degree));
    navX.enableOptionalMessages(true, false, false, false, false, true, false, false, false);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(navX.getYaw().in(Degree));
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(-navX.getAngularVel()[2].in(DegreesPerSecond));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
