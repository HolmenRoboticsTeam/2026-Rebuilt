package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
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
              io.setRollerVoltage(IntakeConstants.rollersMaxVolts);
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
              io.setRollerVoltage(0.0);
            },
            this)
        .withName("Intake_Stop");
  }

  /**
   * Creates and returns a command that folds out the intake.
   *
   * @return A command with the given logic.
   */
  public Command extend() {
    return Commands.runOnce(
            () -> {
              io.setPivotAngle(IntakeConstants.extendAngle);
            },
            this)
        .withName("Intake_Extend");
  }

  /**
   * Creates and returns a command that retracts in the intake.
   *
   * @return A command with the given logic.
   */
  public Command retract() {
    return Commands.runOnce(
            () -> {
              io.setPivotAngle(IntakeConstants.retractAngle);
            },
            this)
        .withName("Intake_Retract");
  }

  /**
   * Get the current position of the pivot
   *
   * @return the position in radians
   */
  public double getPivotPosition() {
    return inputs.pivotPositionRad;
  }

  /**
   * Get the current position of the intake
   *
   * @return the position in rotations
   */
  public double getIntakeRotations() {
    return inputs.rollerPositionRotations;
  }

  /**
   * getIntakeRotations Checks the motor to see if the intake is running.
   *
   * @return Whether the intake is running.
   */
  public boolean isRunning() {
    return !MathUtil.isNear(0.0, inputs.rollerAppliedVolts, 0.1);
  }
}
