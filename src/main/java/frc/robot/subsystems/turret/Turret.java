// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.turret.TurretDistanceCalc.TargetType;
import frc.robot.subsystems.turret.TurretDistanceCalc.TurretShotData;
import frc.robot.subsystems.turret.TurretIO.FlyWheelMode;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A subsystem for controlling the turret, including its three parts: rotation, angle, and flyWheel.
 */
public class Turret extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d> turretPose;

  private Supplier<ChassisSpeeds> robotVelocity;

  private Supplier<Boolean> fedFuel;
  private Consumer<Integer> changeFuelHeld;

  private Debouncer currentControlDebouncer =
      new Debouncer(TurretConstants.currentControlDebounce, DebounceType.kFalling);

  /**
   * Creates a new turret.
   *
   * @param io the implementation of the turret.
   * @param robotPose the pose of the robot.
   * @param robotVelocity the chassis speeds of the robot.
   * @param fedFuel whether the feeder is feeding fuel to the turret.
   */
  public Turret(
      TurretIO io,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotVelocity,
      Supplier<Boolean> fedFuel,
      Consumer<Integer> changeFuelHeld) {
    this.io = io;
    this.robotPose = robotPose;
    turretPose =
        () -> {
          Rotation2d turretRot =
              robotPose
                  .get()
                  .getRotation()
                  .minus(Rotation2d.fromRadians(inputs.rotationPositionRad));
          Translation2d turretTrans =
              robotPose
                  .get()
                  .getTranslation()
                  .plus(
                      new Translation2d(
                          TurretConstants.turretYOffset, TurretConstants.turretXOffset));
          return new Pose2d(turretTrans, turretRot);
        };
    this.robotVelocity = robotVelocity;
    this.fedFuel = fedFuel;
    this.changeFuelHeld = changeFuelHeld;
  }

  private double fuelHz = 0.3;
  private double lastFuel = Timer.getFPGATimestamp();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (Constants.currentMode != Mode.REAL
        && fedFuel.get()
        && lastFuel + fuelHz < Timer.getFPGATimestamp()) {
      io.shootFuel(robotPose.get().getRotation());
      changeFuelHeld.accept(-1);
      lastFuel = Timer.getFPGATimestamp();
    }
  }

  /**
   * Creates and returns a command to fully control the turret's target, rotation, angle, and
   * flyWheel speed. These are controlled based on position of the robot on the field.
   *
   * @return A command with the given logic.
   */
  public Command fullFieldAim() {

    return Commands.runOnce(
            () -> {

              // Get target translation and type based on turret position

              Translation2d turretTarget = getTargetTranslation();
              TargetType targetType = getTargetType();

              // ADJUST THE TARGET BASE ON ROBOT VELOCITY TODO: Mult by time-of-flight
              // turretTarget =
              //     turretTarget.minus(
              //         new Translation2d(
              //             robotVelocity.get().vxMetersPerSecond,
              //             robotVelocity.get().vyMetersPerSecond));

              // AIM THE TURRET AT THE TARGET
              Translation2d deltaTranslation =
                  turretTarget.minus(turretPose.get().getTranslation());
              io.setTargetRotation(
                  deltaTranslation.getAngle().minus(robotPose.get().getRotation()));

              // SPIN AND ANGLE THE FLYWHEEL BASED OFF TURRET DISTANCE FROM TARGET

              Distance distance =
                  Distance.ofRelativeUnits(
                      turretPose.get().getTranslation().getDistance(turretTarget), Meter);

              TurretShotData shotData = TurretDistanceCalc.getShotData(targetType, distance);
              double RPM = shotData.RPM();
              double angle = shotData.angleRad();

              // Set the Flywheel mode

              boolean torqueCurrentControl =
                  currentControlDebouncer.calculate(inputs.flyWheelIsTarget);
              FlyWheelMode flyWheelMode =
                  torqueCurrentControl ? FlyWheelMode.CURRENT : FlyWheelMode.VOLTAGE;

              // Log the outputs

              Logger.recordOutput("Turret/Distance", distance);
              Logger.recordOutput("Turret/Target", turretTarget);
              Logger.recordOutput("Turret/Type", targetType);
              Logger.recordOutput("Turret/FlyWheelMode", flyWheelMode);
              Logger.recordOutput("Turret/RPM", RPM);
              Logger.recordOutput("Turret/Angle", Rotation2d.fromDegrees(angle));
              Logger.recordOutput(
                  "Turret/Rotation",
                  deltaTranslation.getAngle().minus(robotPose.get().getRotation()));
              Logger.recordOutput(
                  "Turret/TrueRotation",
                  deltaTranslation.getAngle().minus(robotPose.get().getRotation()));

              // Set the outputs
              io.setFlyWheelMode(flyWheelMode);
              io.setFlyWheelRPM(RPM);
              io.setTargetAngle(Rotation2d.fromDegrees(angle));
            },
            this)
        .repeatedly()
        .withName("Turret_FullFieldAim");
  }

  /**
   * Creates and returns a command to control the turret with a target position and type.
   *
   * @param turretTarget the target translation
   * @param targetType the target type
   * @return A command with the given logic
   */
  public Command forceAim(Translation2d turretTarget, TargetType type) {

    return Commands.runOnce(
            () -> {
              final TargetType targetType;
              if (getTargetType() == TargetType.INVALID) targetType = TargetType.INVALID;
              else targetType = type;

              // ADJUST THE TARGET BASE ON ROBOT VELOCITY TODO: Mult by time-of-flight
              // turretTarget =
              //     turretTarget.minus(
              //         new Translation2d(
              //             robotVelocity.get().vxMetersPerSecond,
              //             robotVelocity.get().vyMetersPerSecond));

              // AIM THE TURRET AT THE TARGET
              Translation2d deltaTranslation =
                  turretTarget.minus(turretPose.get().getTranslation());
              io.setTargetRotation(
                  deltaTranslation.getAngle().minus(robotPose.get().getRotation()));

              // SPIN AND ANGLE THE FLYWHEEL BASED OFF TURRET DISTANCE FROM TARGET

              Distance distance =
                  Distance.ofRelativeUnits(
                      turretPose.get().getTranslation().getDistance(turretTarget), Meter);

              TurretShotData shotData = TurretDistanceCalc.getShotData(targetType, distance);
              double RPM = shotData.RPM();
              double angle = shotData.angleRad();

              // Set the Flywheel mode

              boolean torqueCurrentControl =
                  currentControlDebouncer.calculate(inputs.flyWheelIsTarget);
              FlyWheelMode flyWheelMode =
                  torqueCurrentControl ? FlyWheelMode.CURRENT : FlyWheelMode.VOLTAGE;

              // Log the outputs

              Logger.recordOutput("Turret/Target", turretTarget);
              Logger.recordOutput("Turret/Type", targetType);
              Logger.recordOutput("Turret/FlyWheelMode", flyWheelMode);
              Logger.recordOutput("Turret/RPM", RPM);
              Logger.recordOutput("Turret/Angle", Rotation2d.fromDegrees(angle));
              Logger.recordOutput(
                  "Turret/Rotation",
                  deltaTranslation.getAngle().minus(robotPose.get().getRotation()));

              // Set the outputs
              io.setFlyWheelMode(flyWheelMode);
              io.setFlyWheelRPM(RPM);
              io.setTargetAngle(Rotation2d.fromDegrees(angle));
            },
            this)
        .repeatedly()
        .withName("Turret_FullFieldAim");
  }

  /**
   * Creates and returns a command to pull RPM and Angle of the shooter from NetworkTables for
   * faster tuning.
   *
   * @return A command with the given logic.
   */
  public Command calibrate() {

    LoggedNetworkNumber tunableRPM = new LoggedNetworkNumber("/Tuning/RPM", 0.0);
    LoggedNetworkNumber tunableAngle = new LoggedNetworkNumber("/Tuning/Angle", 0.0);

    return Commands.runOnce(
            () -> {
              io.setTargetRotation(Rotation2d.kZero);

              double RPM = tunableRPM.get();
              double angle = tunableAngle.get();

              // Set the Flywheel mode

              boolean torqueCurrentControl =
                  currentControlDebouncer.calculate(inputs.flyWheelIsTarget);
              FlyWheelMode flyWheelMode =
                  torqueCurrentControl ? FlyWheelMode.CURRENT : FlyWheelMode.VOLTAGE;

              // Log the outputs

              Logger.recordOutput("Turret/FlyWheelMode", flyWheelMode);
              Logger.recordOutput("Turret/RPM", RPM);
              Logger.recordOutput("Turret/Angle", Rotation2d.fromDegrees(angle));

              // Set the outputs
              io.setFlyWheelMode(flyWheelMode);
              io.setFlyWheelRPM(RPM);
              io.setTargetAngle(Rotation2d.fromDegrees(angle));
            },
            this)
        .withName("Turret_Calibrate");
  }

  /**
   * Creates and returns a command to center the rotation, stop the flyWheel, and drop the angle.
   *
   * @return A command with the given logic.
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setTargetRotation(Rotation2d.kZero);
              io.setFlyWheelRPM(0.0);
              io.setTargetAngle(Rotation2d.kZero);
            },
            this)
        .withName("Turret_Stop");
  }

  public double getRotation() {
    return inputs.rotationPositionRad;
  }

  public double getAngle() {
    return inputs.anglePositionRad;
  }

  public double getFlyWheelPosition() {
    return inputs.flyWheelPositionRotations;
  }

  /**
   * Checks to see if the rotation, angle, and flyWheel are at their targets.
   *
   * @return Whether all three are at their target.
   */
  public boolean isReadyForFuel() {
    return inputs.rotationIsAtTarget && inputs.angleIsAtTarget && inputs.flyWheelIsTarget;
  }

  /**
   * Finds and returns which target position is correct based on the position of the robot.
   *
   * @return The position of the target.
   */
  private Translation2d getTargetTranslation() {
    if (FieldConstants.kHomeAllianceZone.contains(
        turretPose.get().getTranslation())) { // In Alliance Zone
      return FieldConstants.kHubPosition;

    } else if (FieldConstants.kLeftNeutralSide.contains(
        turretPose.get().getTranslation())) { // On Left Side
      return FieldConstants.kLeftCorner;

    } else if (FieldConstants.kRightNeutralSide.contains(
        turretPose.get().getTranslation())) { // On Right Side
      return FieldConstants.kRightCorner;

    } else { // Ok, the turret pose is outside the field, yay! // TODO: do a better check
      return FieldConstants.kHubPosition;
    }
  }

  /**
   * Find and returns which target type is correct based on the position of th e robot.
   *
   * @return The type of the target.
   */
  private TargetType getTargetType() {
    if (FieldConstants.kHomeAllianceZone.contains(
        turretPose.get().getTranslation())) { // In Alliance Zone
      return TargetType.HUB;

    } else if (FieldConstants.kLeftNeutralSide.contains(
        turretPose.get().getTranslation())) { // On Left Side
      return TargetType.GROUND;

    } else if (FieldConstants.kRightNeutralSide.contains(
        turretPose.get().getTranslation())) { // On Right Side
      return TargetType.GROUND;

    } else {
      return TargetType.INVALID;
    }
  }
}
