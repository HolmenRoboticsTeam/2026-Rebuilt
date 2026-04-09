// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretDistanceCalc.TargetType;
import frc.robot.util.HubShiftUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the feeder with a line-break to detect fuel inside. */
public class Feeder extends SubsystemBase {

  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private Supplier<Boolean> turretIsReady;
  private Supplier<TargetType> targetType;

  /**
   * Creates a new feeder.
   *
   * @param io the implementation of the feeder.
   */
  public Feeder(FeederIO io, Supplier<Boolean> turretIsReady, Supplier<TargetType> targetType) {
    this.io = io;
    this.turretIsReady = turretIsReady;
    this.targetType = targetType;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  /**
   * Creates and returns a command that controls the feeder automatically base on the position on
   * the field, the active shift, and whether the turret is ready. If target is hub and inactive
   * shift do not feed, overwise feed. As well only shoot when the turret is ready.
   *
   * <p>TODO: This command always acted funny, more testing is needed.
   *
   * @return A command with the given logic
   */
  public Command autoFeed() {
    return Commands.repeatingSequence(
            stop(),
            Commands.waitUntil(
                () ->
                    (turretIsReady.get()
                            && (targetType.get().equals(TargetType.HUB)
                                ? HubShiftUtil.getShiftedShiftInfo().active()
                                : true))
                        || !hasExitFuel()),
            start(),
            Commands.waitUntil(
                () ->
                    !(turretIsReady.get()
                            && (targetType.get().equals(TargetType.HUB)
                                ? HubShiftUtil.getShiftedShiftInfo().active()
                                : true))
                        && hasExitFuel()))
        .finallyDo(() -> io.setVolts(0.0))
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
   * Creates and returns a command that reverse the feeder.
   *
   * @return A command with the given logic.
   */
  public Command reverse() {
    return Commands.runOnce(
            () -> {
              io.setVolts(-FeederConstants.maxVolts);
            })
        .withName("Feeder_Reverse");
  }

  /**
   * Get the current position of the feeder
   *
   * @return the position in rotations
   */
  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * Checks the motor to see if the feeder to releasing fuel to the turret
   *
   * @return Whether the feeder is releasing fuel.
   */
  public boolean feedingFuel() {
    return inputs.releasingFuel;
  }

  /**
   * Checks if the feeder has fuel at the enter linebreak
   *
   * @return Whether the feeder has fuel inside.
   */
  public boolean hasEnterFuel() {
    return inputs.hasEnterFuel;
  }

  /**
   * Checks if the feeder has fuel at the enter linebreak
   *
   * @return Whether the feeder has fuel inside.
   */
  public boolean hasExitFuel() {
    return inputs.hasExitFuel;
  }
}
