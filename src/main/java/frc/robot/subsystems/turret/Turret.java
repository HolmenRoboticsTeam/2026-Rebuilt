// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final Supplier<Pose2d> drivePose;
  private final TurretIO io;

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> drivePose) {
    this.drivePose = drivePose;
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command fullFieldAuto() {
    return Commands.run(
        () -> {
          double deltaX =
              FieldConstants.kHubPosition.getMeasureX().in(Meters)
                  - drivePose.get().getMeasureX().in(Meters);
          double deltaY =
              FieldConstants.kHubPosition.getMeasureY().in(Meters)
                  - drivePose.get().getMeasureY().in(Meters);
          double targetRotation =
              Math.atan2(deltaY, deltaX) - drivePose.get().getRotation().getRadians();

          double distance = Math.hypot(deltaX, deltaY);
          double targetAngle = TurretDIstanceCalc.getAngle(distance);
          double targetRPM = TurretDIstanceCalc.getRPM(distance);

          io.setTargetRotation(Rotation2d.fromRadians(targetRotation));
          io.setTargetAngle(Rotation2d.fromRadians(targetAngle));
          io.setFlyWheelRPM(targetRPM);
        },
        this);
  }
}
