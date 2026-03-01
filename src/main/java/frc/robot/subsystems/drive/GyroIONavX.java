// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

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
        SparkOdometryThread.getInstance().registerSignal(this::getAngle);
    navX.enableOptionalMessages(true, true, false, false, false, false, true, false, false, false);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    double angleDeg = getAngle();
    if (Double.isFinite(angleDeg)) {
      inputs.connected = true;
      inputs.yawPosition = Rotation2d.fromDegrees(angleDeg);
    } else {
      inputs.connected = false;
    }
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(navX.getAngularVel()[2].in(DegreesPerSecond));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  /**
   * Gets the countinous angle from the IMU filtering algorithm.
   *
   * Unlike {@link com.studica.frc.Navx#getYaw()}, angle reported
   * by this method is unbounded.
   *
   * Applies a 180deg offset if on Red alliance. Assumes robot boots
   * while facing Blue alliance wall.
   *
   * @return The angle with the offset applied if on Red alliance.
   */
  private double getAngle() {
    double rawAngle = navX.getAngle().in(Degrees);
    return rawAngle + (Constants.isBlueAlliance.get() ? 0.0 : 180.0);
  }
}
