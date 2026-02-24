package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the hopper. */
public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  // This should include fuel held in the intake, hopper, indexer, and feeder.
  private int fuelHeld = 8;

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

  /** Creates and returns a command that stops the hopper. */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this)
        .withName("Hopper_Stop");
  }

  /**
   * Get the current position of the hopper
   *
   * @return the position in rotations
   */
  public double getPosition() {
    return inputs.positionRotations;
  }

  public void changeFuelCountBy(int delta) {
    fuelHeld += delta;
    Logger.recordOutput("Hopper/FuelHeld", fuelHeld);
  }

  public int getHeldFuel() {
    return fuelHeld;
  }
}
