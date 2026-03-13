// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Consumer<Integer> changeHeldFuelBy;

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
      Consumer<Integer> changeHeldFuelBy) {
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
                              TurretConstants.turretXOffset, TurretConstants.turretYOffset)
                          .rotateBy(robotPose.get().getRotation()));
          return new Pose2d(turretTrans, turretRot);
        };
    this.robotVelocity = robotVelocity;
    this.fedFuel = fedFuel;
    this.changeHeldFuelBy = changeHeldFuelBy;
  }

  private double fuelHz = 0.2;
  private double lastFuel = Timer.getFPGATimestamp();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (Constants.currentMode == Mode.SIM
        && fedFuel.get()
        && lastFuel + fuelHz < Timer.getFPGATimestamp()) {
      io.shootFuel(robotPose.get().getRotation());
      changeHeldFuelBy.accept(-1);
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

    return Commands.run(
            () -> {

              // Get target translation and type based on turret position
              Translation2d turretTarget = getTargetTranslation();
              // TargetType targetType = getTargetType();
              TargetType targetType = TargetType.HUB;

              // Get the shot data
              Distance distance =
                  Meters.of(turretPose.get().getTranslation().getDistance(turretTarget));
              TurretShotData shotData = TurretDistanceCalc.getShotData(targetType, distance);

              // Adjust target based on robot movement times time of flight.
              turretTarget =
                  turretTarget.minus(
                      new Translation2d(
                              robotVelocity.get().vxMetersPerSecond,
                              robotVelocity.get().vyMetersPerSecond)
                          .times(shotData.timeOfFlightSec()));

              // Update shot data
              distance = Meters.of(turretPose.get().getTranslation().getDistance(turretTarget));
              shotData = TurretDistanceCalc.getShotData(targetType, distance);
              /**
               * TLDR: The time of flight data should not be updated.
               *
               * <p>The shot data needs to be update because me moved the target based on the
               * velocity of the robot. The time of flight does not need to be updated because it is
               * not affect by the robot's velocity. Why? The robot's velocity is only horizontal
               * AND the time of flight to only based on when the fuel hit the target from above,
               * its vertical position. So regardless of the velocity of the robot, the fuel's
               * vertical velocity will not be affected. Therefore, the time of flight will remain
               * unchanged.
               */

              // Get the target values
              Rotation2d rotation =
                  turretTarget
                      .minus(turretPose.get().getTranslation())
                      .getAngle()
                      .minus(robotPose.get().getRotation());
              Rotation2d angle = Rotation2d.fromRadians(shotData.angleRad());
              AngularVelocity rpm = RPM.of(shotData.RPM());

              // Set the Flywheel mode
              // boolean torqueCurrentControl =
              //     currentControlDebouncer.calculate(inputs.flyWheelIsTarget);
              // FlyWheelMode flyWheelMode =
              //     torqueCurrentControl ? FlyWheelMode.CURRENT : FlyWheelMode.VOLTAGE;
              FlyWheelMode flyWheelMode = FlyWheelMode.VOLTAGE;

              // Log the outputs
              Logger.recordOutput("Turret/Distance", distance);
              Logger.recordOutput("Turret/Target", turretTarget);
              Logger.recordOutput("Turret/Type", targetType);
              Logger.recordOutput("Turret/FlyWheelMode", flyWheelMode);
              Logger.recordOutput("Turret/RPM", rpm);
              Logger.recordOutput("Turret/Angle", angle);
              Logger.recordOutput("Turret/Rotation", rotation);

              // Set the outputs
              io.setTargetRotation(rotation.plus(Rotation2d.k180deg));
              io.setTargetAngle(angle);
              io.setFlyWheelMode(flyWheelMode);
              io.setFlyWheelRPM(rpm.in(RPM));
            },
            this)
        .withName("Turret_FullFieldAim");
  }

  /**
   * Creates and returns a command to pull RPM and Angle of the shooter from NetworkTables for
   * faster tuning.
   *
   * @return A command with the given logic.
   */
  public Command calibrate() {

    SmartDashboard.putNumber("RPM", 0.0);
    SmartDashboard.putNumber("Angle", 0.0);

    return Commands.run(
            () -> {

              // Get target translation and type based on turret position
              Translation2d turretTarget = getTargetTranslation();
              // TargetType targetType = getTargetType();
              TargetType targetType = TargetType.HUB;

              // Get distance and rotation for logging
              Distance distance =
                  Meters.of(turretPose.get().getTranslation().getDistance(turretTarget));
              Rotation2d rotation =
                  turretTarget
                      .minus(turretPose.get().getTranslation())
                      .getAngle()
                      .minus(robotPose.get().getRotation());

              // Pull Tunable values
              double rpm = SmartDashboard.getNumber("RPM", 0.0);
              Rotation2d angle = Rotation2d.fromRadians(SmartDashboard.getNumber("Angle", 0.0));

              // Set the Flywheel mode
              // boolean torqueCurrentControl =
              //     currentControlDebouncer.calculate(inputs.flyWheelIsTarget);
              // FlyWheelMode flyWheelMode =
              //     torqueCurrentControl ? FlyWheelMode.CURRENT : FlyWheelMode.VOLTAGE;
              FlyWheelMode flyWheelMode = FlyWheelMode.VOLTAGE;

              // Log the outputs
              Logger.recordOutput("Turret/Distance", distance);
              Logger.recordOutput("Turret/Target", turretTarget);
              Logger.recordOutput("Turret/Type", targetType);
              Logger.recordOutput("Turret/FlyWheelMode", flyWheelMode);
              Logger.recordOutput("Turret/RPM", rpm);
              Logger.recordOutput("Turret/Angle", angle);
              Logger.recordOutput("Turret/Rotation", rotation);

              // Set the outputs
              io.setTargetRotation(rotation);
              io.setTargetAngle(angle);
              io.setFlyWheelMode(flyWheelMode);
              io.setFlyWheelRPM(rpm);
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

  private final int totalSteps = 3;
  private final double stepTime = 0.2;
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  /**
   * Find and returns which target type is correct based on the position and speed of the robot.
   *
   * @return The type of the target.
   */
  private TargetType getTargetType() {

    ChassisSpeeds speeds = robotVelocity.get();
    Translation2d turretTrans = turretPose.get().getTranslation();

    // Take the difference in the angles of the current and last velocity, then scale it to predict
    // the path of the robot.
    Rotation2d turnPredictRot =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .getAngle()
            .minus(
                new Translation2d(lastSpeeds.vxMetersPerSecond, lastSpeeds.vyMetersPerSecond)
                    .getAngle())
            .times(3.2);

    // Used for logging the predicted path of the robot.
    Translation2d[] steps = new Translation2d[totalSteps];
    for (int i = 0; i < totalSteps; i++) {
      steps[i] = turretTrans;
    }

    // Loop each step.
    for (int stepNum = 1; stepNum < totalSteps; stepNum++) {

      // The amount of time into the future to predict.
      double time = stepNum * stepTime;

      // The actual prediction step.
      // Turret position + (velocity * time). Then rotation by the angle to get field relative, then
      // rotate by the turn prediction to get the predicted path of the robot.
      Translation2d step =
          turretTrans.plus(
              new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                  .times(time)
                  .rotateBy(robotPose.get().getRotation())
                  .rotateBy(turnPredictRot.times(stepNum)));

      // Move any steps into the field to allow for better tracking, and then check the type of the
      // step.
      step = forceInField(step);
      TargetType stepType = checkTrans(step);
      // Log the step.
      steps[totalSteps - stepNum - 1] = step;

      // If the step lands in the trench, return IN_TRENCH.
      if (stepType == TargetType.IN_TRENCH) {
        for (int i = stepNum + 1; i < totalSteps; i++) {
          steps[totalSteps - i - 1] = step;
        }

        Logger.recordOutput("Turret/Steps", steps);
        lastSpeeds = speeds;
        return TargetType.IN_TRENCH;
      }
    }

    // Log all the steps
    Logger.recordOutput("Turret/Steps", steps);

    // Otherwise, do the normal check.
    lastSpeeds = speeds;
    return checkTrans(turretTrans);
  }

  /**
   * Return which targetType the given translation should target.
   *
   * @param trans the given translation
   * @return Which targetType
   */
  private TargetType checkTrans(Translation2d trans) {
    if (FieldConstants.kHomeAllianceZone.contains(trans)) { // In Alliance Zone
      return TargetType.HUB;

    } else if (FieldConstants.kLeftNeutralSide.contains(trans)
        || FieldConstants.kLeftOpposingSide.contains(trans)) { // On Left Side
      return TargetType.GROUND;

    } else if (FieldConstants.kRightNeutralSide.contains(trans)
        || FieldConstants.kRightOpposingSide.contains(trans)) { // On Right Side
      return TargetType.GROUND;

    } else if (FieldConstants.kAllianceTrenchBumpZone.contains(trans)
        || FieldConstants.kOpposingTrenchBumpZone.contains(trans)) { // In the trench/On the bump
      return TargetType.IN_TRENCH;

    } else { // Out of the field
      return TargetType.INVALID;
    }
  }

  public Translation2d forceInField(Translation2d trans) {

    if (FieldConstants.kWholeField.contains(trans)) return trans;

    Distance xDistance;
    Distance yDistance;

    if (trans.getX() < 0.0) {
      xDistance = Inches.of(0.0);
    } else if (trans.getMeasureX().compareTo(FieldConstants.fieldLength) > 0) {
      xDistance = FieldConstants.fieldLength.copy();
    } else {
      xDistance = trans.getMeasureX();
    }

    if (trans.getY() < 0.0) {
      yDistance = Inches.of(0.0);
    } else if (trans.getMeasureY().compareTo(FieldConstants.fieldWidth) > 0) {
      yDistance = FieldConstants.fieldWidth.copy();
    } else {
      yDistance = trans.getMeasureY();
    }

    return new Translation2d(xDistance, yDistance);
  }
}
