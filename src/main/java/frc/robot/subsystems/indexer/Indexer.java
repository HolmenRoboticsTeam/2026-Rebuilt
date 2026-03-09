package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the indexer. */
public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private Supplier<Boolean> feederHasFuel;
  private Debouncer hasFuelDebouncer = new Debouncer(IndexerConstants.hasFuelDebouncerTime, DebounceType.kFalling);
  private int fuelHeldCount = 0;

  /**
   * Creates a new indexer.
   *
   * @param io the implementation of the indexer.
   */
  public Indexer(IndexerIO io, Supplier<Boolean> feederHasFuel) {
    this.io = io;
    this.feederHasFuel = feederHasFuel;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    // This is bad, I don't care
    if (Constants.currentMode != Mode.REAL) inputs.hasFuel = fuelHeldCount > 0;
    Logger.processInputs("Indexer", inputs);

    Logger.recordOutput("Indexer/HeldFuelCount", fuelHeldCount);
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
   * Creates and returns a command that runs the indexer.
   *
   * @return A command with the given logic.
   */
  public Command start() {
    return Commands.runOnce(
            () -> {
              io.setVolts(IndexerConstants.maxVolts);
            },
            this)
        .withName("Indexer_Start");
  }

  /**
   * Creates and returns a command that stops the indexer.
   *
   * @return A command with given logic.
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this)
        .withName("Indexer_Stop");
  }

  /**
   * Get the current position of the indexer
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
