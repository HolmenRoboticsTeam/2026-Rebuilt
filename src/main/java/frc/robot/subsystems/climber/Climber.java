// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling a single climber with a line-break as a hard-stop. */
public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /**
   * Creates a new climber.
   *
   * @param io the implementation of the climber.
   */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /**
   * Creates and returns a command that extends the climber to max height.
   *
   * @return A command with the given logic.
   */
  public Command extend() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setVolts(ClimberConstants.extendVolts);
            },
            this),
        Commands.waitUntil(() -> inputs.positionMeters >= ClimberConstants.extendHeight),
        Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this));
  }

  /**
   * Creates and returns a command that retracts the climber to the hard-stop.
   *
   * @return A command with the given logic.
   */
  public Command retract() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setVolts(ClimberConstants.retractVolts);
            },
            this),
        Commands.waitUntil(() -> inputs.hardStop),
        Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this));
  }

  /**
   * Creates and returns a command that retracts the climber to the hard-stop, then resets the
   * position.
   *
   * @return
   */
  public Command calibrate() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              io.setVolts(ClimberConstants.retractVolts);
            },
            this),
        Commands.waitUntil(() -> inputs.hardStop),
        Commands.runOnce(
            () -> {
              io.setVolts(0.0);
              io.resetPosition();
            },
            this));
  }
}
