package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the indexer. */
public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /**
   * Creates a new indexer.
   *
   * @param io the implementation of the indexer.
   */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
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
}
