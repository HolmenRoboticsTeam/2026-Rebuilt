package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** A subsystem for controlling the intake. */
public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /**
   * Creates a new intake.
   *
   * @param io the implementation of the intake.
   */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Creates and returns a command that runs the intake.
   *
   * @return A command with the given logic.
   */
  public Command start() {
    return Commands.runOnce(
            () -> {
              io.setVolts(IntakeConstants.maxVolts);
            },
            this)
        .withName("Intake_Start");
  }

  /**
   * Creates and returns a command that stops the intake.
   *
   * @return A command with the given logic.
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              io.setVolts(0.0);
            },
            this)
        .withName("Intake_Stop");
  }

  /**
   * Checks the motor to see if the intake is running.
   *
   * @return Whether the intake is running.
   */
  public boolean isRunning() {
    return inputs.isRunning;
  }
}
