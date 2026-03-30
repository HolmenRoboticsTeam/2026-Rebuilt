// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/** Add your docs here. */
public class StateLoggingCommands {

  private static double canvasWidth = 1.0;
  private static double canvasHeight = 1.0;

  private static LoggedMechanism2d intakeMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d intakeLigament =
      new LoggedMechanismLigament2d("Intake", 0.1, 0.0);

  private static LoggedMechanism2d indexerMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d indexerLigament =
      new LoggedMechanismLigament2d("indexer", 0.1, 0.0);

  private static LoggedMechanism2d feederMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d feederLigament =
      new LoggedMechanismLigament2d("riser", 0.1, 0.0);

  private static LoggedMechanism2d turretMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d angleLigament =
      new LoggedMechanismLigament2d("angle", 0.1, 0.0);
  private static LoggedMechanismLigament2d flyWheelLigament =
      new LoggedMechanismLigament2d("flyWheel", 0.05, 0.0);

  static {
    intakeMechanism.getRoot("root", 1.0, 0.2).append(intakeLigament);
    intakeLigament.setColor(new Color8Bit(0, 0, 255));
    intakeLigament.setLineWeight(1.0);

    indexerMechanism.getRoot("root", 0.4, 0.4).append(indexerLigament);
    indexerLigament.setColor(new Color8Bit(255, 0, 0));
    indexerLigament.setLineWeight(1.0);

    feederMechanism.getRoot("root", 0.2, 0.3).append(feederLigament);
    feederLigament.setColor(new Color8Bit(0, 255, 0));
    feederLigament.setLineWeight(1.0);

    turretMechanism.getRoot("root", 0.2, 0.4).append(angleLigament).append(flyWheelLigament);
    angleLigament.setColor(new Color8Bit(255, 0, 255));
    flyWheelLigament.setColor(new Color8Bit(0, 255, 255));
    angleLigament.setLineWeight(2.5);
    flyWheelLigament.setLineWeight(1.0);
  }

  public static Command logMechanisms(
      Intake intake, Indexer indexer, Feeder feeder, Turret turret) {
    return Commands.run(
            () -> {
              intakeLigament.setAngle(Rotation2d.fromRotations(intake.getPosition()));
              indexerLigament.setAngle(Rotation2d.fromRotations(indexer.getPosition()));
              feederLigament.setAngle(Rotation2d.fromRotations(feeder.getPosition()));

              angleLigament.setAngle(Rotation2d.fromRadians(turret.getAngle()));
              flyWheelLigament.setAngle(Rotation2d.fromRotations(turret.getFlyWheelPosition()));

              Logger.recordOutput("Mechanism/Intake", intakeMechanism);
              Logger.recordOutput("Mechanism/Indexer", indexerMechanism);
              Logger.recordOutput("Mechanism/Feeder", feederMechanism);

              Logger.recordOutput("Mechanism/Turret", turretMechanism);
            })
        .ignoringDisable(true)
        .withName("logMechanisms");
  }

  public static Command updateDashboard() {
    return Commands.run(
            () -> {
              SmartDashboard.putNumber(
                  "Remaining Time In Current Shift",
                  HubShiftUtil.getOfficialShiftInfo().remainingTime());
              SmartDashboard.putBoolean(
                  "Our Hub is Active?", HubShiftUtil.getOfficialShiftInfo().active());
              SmartDashboard.putString(
                  "Current Shift", HubShiftUtil.getOfficialShiftInfo().currentShift().name());
              SmartDashboard.putString(
                  "Auto Winner",
                  HubShiftUtil.getFirstActiveAlliance() == Alliance.Blue ? "Red" : "Blue");
            })
        .ignoringDisable(true)
        .withName("updateDashboard");
  }

  public static Command rumbleOnShiftChange(CommandXboxController controller) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)),
            Commands.waitUntil(() -> HubShiftUtil.getOfficialShiftInfo().remainingTime() < 5),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.5)),
            Commands.waitUntil(() -> HubShiftUtil.getOfficialShiftInfo().remainingTime() > 5))
        .withName("Controller_ShiftRumbleCommand");
  }
}
