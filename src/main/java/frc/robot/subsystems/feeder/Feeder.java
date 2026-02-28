// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the feeder with a line-break to detect fuel inside. */
public class Feeder extends SubsystemBase {

  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private Supplier<Boolean> turretIsReady;

  /**
   * Creates a new feeder.
   *
   * @param io the implementation of the feeder.
   */
  public Feeder(FeederIO io, Supplier<Boolean> turretIsReady) {
    this.io = io;
    this.turretIsReady = turretIsReady;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public Command autoFeed() {
    return Commands.repeatingSequence(
            stop(),
            Commands.waitUntil(() -> turretIsReady.get()),
            start(),
            Commands.waitUntil(() -> !turretIsReady.get()).finallyDo(() -> io.setVolts(0.0)))
        .withName("Feeder_Auto");
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

  public Command reverse() {
    return Commands.runOnce(
            () -> {
              io.setVolts(-FeederConstants.maxVolts);
            })
        .withName("Feeder_Reverse");
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
   * Get the current position of the feeder
   *
   * @return the position in rotations
   */
  public double getPosition() {
    return inputs.positionRotations;
  }

  public boolean hasFuel() {
    return inputs.hasFuel;
  }

  /**
   * Checks the motor to see if the feeder to releasing fuel to the turret and if the feeder has
   * fuel.
   *
   * @return Whether the feeder has and is releasing fuel.
   */
  public boolean feedingFuel() {
    return inputs.releasingFuel && inputs.hasFuel;
  }
}
