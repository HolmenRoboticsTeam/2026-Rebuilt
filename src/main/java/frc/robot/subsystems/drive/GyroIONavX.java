// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kelvin;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;
import java.util.function.Supplier;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {

  private final Navx navX =
      new Navx(DriveConstants.navXCanId, (int) DriveConstants.odometryFrequency);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Supplier<Double> adjustedYaw =
      () ->
          Constants.isBlueAlliance.get() ? 0.0 - Math.toDegrees(0.1) : 180.0 - Math.toDegrees(0.1);

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> (navX.getYaw().in(Degree) + adjustedYaw.get()) % 360.0);
    navX.enableOptionalMessages(true, false, false, false, false, false, true, false, false, true);
    navX.resetYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.getTemperature().in(Kelvin) != 0.0;
    inputs.yawPosition =
        Rotation2d.fromDegrees((navX.getYaw().in(Degree) + adjustedYaw.get()) % 360.0);
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(navX.getAngularVel()[2].in(DegreesPerSecond));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            // Do not adjust this value, it is being adjusting in the constructor.
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
