// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/** Add your docs here. */
public class StateLoggingCommands {

  private static double canvasWidth = 1.0;
  private static double canvasHeight = 1.0;

  private static LoggedMechanism2d intakeMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d intakeAngleLigament =
      new LoggedMechanismLigament2d("Intake Angle", 0.1, 0.0);
  private static LoggedMechanismLigament2d intakeLigament =
      new LoggedMechanismLigament2d("Intake", 0.1, 0.0);

  private static LoggedMechanism2d hopperMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d hopperLigament =
      new LoggedMechanismLigament2d("Hopper", 0.1, 0.0);

  private static LoggedMechanism2d feederMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d feederLigament =
      new LoggedMechanismLigament2d("Feeder", 0.1, 0.0);

  private static LoggedMechanism2d turretMechanism =
      new LoggedMechanism2d(canvasWidth, canvasHeight);
  private static LoggedMechanismLigament2d turretAngleLigament =
      new LoggedMechanismLigament2d("Turret Angle", 0.1, 0.0);
  private static LoggedMechanismLigament2d flyWheelLigament =
      new LoggedMechanismLigament2d("Fly Wheel", 0.05, 0.0);

  static {
    intakeMechanism.getRoot("root", 1.0, 0.2).append(intakeAngleLigament).append(intakeLigament);
    intakeAngleLigament.setColor(new Color8Bit(0, 255, 0));
    intakeLigament.setColor(new Color8Bit(0, 0, 255));
    intakeAngleLigament.setLineWeight(1.0);
    intakeLigament.setLineWeight(1.0);

    hopperMechanism.getRoot("root", 0.4, 0.4).append(hopperLigament);
    hopperLigament.setColor(new Color8Bit(255, 0, 0));
    hopperLigament.setLineWeight(1.0);

    feederMechanism.getRoot("root", 0.2, 0.3).append(feederLigament);
    feederLigament.setColor(new Color8Bit(0, 255, 0));
    feederLigament.setLineWeight(1.0);

    turretMechanism.getRoot("root", 0.2, 0.4).append(turretAngleLigament).append(flyWheelLigament);
    turretAngleLigament.setColor(new Color8Bit(255, 0, 255));
    flyWheelLigament.setColor(new Color8Bit(0, 255, 255));
    turretAngleLigament.setLineWeight(2.5);
    flyWheelLigament.setLineWeight(1.0);
  }

  public static Command logMechanisms(Intake intake, Hopper hopper, Feeder feeder, Turret turret) {
    return Commands.run(
            () -> {
              intakeAngleLigament.setAngle(
                  Rotation2d.fromRadians(intake.getPivotPosition())
                      .times(-1)
                      .plus(Rotation2d.kCCW_90deg));
              intakeLigament.setAngle(Rotation2d.fromRotations(intake.getIntakeRotations()));

              hopperLigament.setAngle(Rotation2d.fromRotations(hopper.getPosition()));
              feederLigament.setAngle(Rotation2d.fromRotations(feeder.getPosition()));

              turretAngleLigament.setAngle(Rotation2d.fromRadians(turret.getAngle()));
              flyWheelLigament.setAngle(Rotation2d.fromRotations(turret.getFlyWheelPosition()));

              Logger.recordOutput("Mechanism/Intake", intakeMechanism);
              Logger.recordOutput("Mechanism/Hopper", hopperMechanism);
              Logger.recordOutput("Mechanism/Feeder", feederMechanism);

              Logger.recordOutput("Mechanism/Turret", turretMechanism);
            })
        .ignoringDisable(true);
  }
}
