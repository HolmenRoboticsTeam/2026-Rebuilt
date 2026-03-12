package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the hopper. */
public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private Supplier<Boolean> feederHasFuel;
  private Debouncer hasFuelDebouncer =
      new Debouncer(HopperConstants.hasFuelDebouncerTime, DebounceType.kFalling);
  private int fuelHeldCount = 0;

  /**
   * Creates a new hopper.
   *
   * @param io the implementation of the hopper.
   */
  public Hopper(HopperIO io, Supplier<Boolean> feederHasFuel) {
    this.io = io;
    this.feederHasFuel = feederHasFuel;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    // This is bad, I don't care
    if (Constants.currentMode != Mode.REAL) inputs.hasFuel = fuelHeldCount > 0;
    Logger.processInputs("Hopper", inputs);

    Logger.recordOutput("Hopper/HeldFuelCount", fuelHeldCount);
  }

  public Command autoIndex() {
    return Commands.repeatingSequence(
            stop(),
            Commands.waitUntil(() -> !feederHasFuel.get()),
            start(),
            Commands.waitUntil(() -> feederHasFuel.get()))
        .finallyDo(() -> io.setVolts(0.0));
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
   * Get the current position of the hopper
   *
   * @return the position in rotations
   */
  public double getPosition() {
    return inputs.positionRotation;
  }

  public boolean hasFuel() {
    return hasFuelDebouncer.calculate(inputs.hasFuel);
  }

  public void changeHeldFuelBy(int delta) {
    this.fuelHeldCount += delta;
  }

  public int getHeldFuel() {
    return this.fuelHeldCount;
  }
}
