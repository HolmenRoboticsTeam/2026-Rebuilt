// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the feeder with a line-break to detect fuel inside. */
public class Feeder extends SubsystemBase {

  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  /**
   * Creates a new feeder.
   *
   * @param io the implementation of the feeder.
   */
  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  /**
   * Creates and returns a command that runs the feeder.
   *
   * @return A command with the given logic.
   */
  public Command start() {
    return Commands.runOnce(
            () -> {
              io.setVolts(FeederConstants.maxVolts);
            },
            this)
        .withName("Feeder_Start");
  }

  /**
   * Creates and returns a command that stops the feeder.
   *
   * @return A command with the given logic.
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this)
        .withName("Feeder_Stop");
  }

  /**
   * Check whether the line-break is broken to see if there is fuel inside the feeder.
   *
   * @return Whether there is fuel inside the feeder.
   */
  public boolean hasFuel() {
    return inputs.hasFuel;
  }

  /**
   * Checks the motor to see if the feeder to releasing fuel to the turret.
   *
   * @return Whether the feeder is releasing fuel.
   */
  public boolean releasingFuel() {
    return inputs.releasingFuel;
  }
}
