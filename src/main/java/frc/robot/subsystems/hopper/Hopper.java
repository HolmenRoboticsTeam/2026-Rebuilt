package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the hopper. */
public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  /**
   * Creates a new hopper.
   *
   * @param io the implementation of the hopper.
   */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  /**
   * Creates and returns a command to automatically run the hopper's rollers. Reversing for one
   * second, to remove jams, and then running forward for 3 seconds.
   *
   * @return A command with the given logic
   */
  public Command autoHop() {
    return Commands.repeatingSequence(
            reverse(), Commands.waitSeconds(1.0), start(), Commands.waitSeconds(3.0))
        .withName("Hopper_Auto");
  }

  /**
   * Creates and returns a command that runs the hopper.
   *
   * @return A command with the given logic.
   */
  public Command start() {
    return Commands.runOnce(
            () -> {
              io.setVolts(HopperConstants.maxVolts);
            },
            this)
        .withName("Hopper_Start");
  }

  /**
   * Creates and returns a command that stops the hopper.
   *
   * @return A command with given logic.
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this)
        .withName("Hopper_Stop");
  }

  /**
   * Creates and returns a command that reverse the hopper.
   *
   * @return A command with the given logic.
   */
  public Command reverse() {
    return Commands.runOnce(
            () -> {
              io.setVolts(-HopperConstants.maxVolts);
            },
            this)
        .withName("Hopper_Reverse");
  }

  /**
   * Get the current position of the hopper
   *
   * @return the position in rotations
   */
  public double getPosition() {
    return inputs.positionRotation;
  }

  public boolean hasFuel() {
    return inputs.hasFuel;
  }
}
