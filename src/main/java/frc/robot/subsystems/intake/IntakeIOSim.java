// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the intake */
public class IntakeIOSim implements IntakeIO {

  private DCMotorSim intakeMotor;

  private double appliedVolts;

  /** Creates a new sim intake. */
  public IntakeIOSim() {

    intakeMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Sim.motorGearBox,
                IntakeConstants.Sim.JKgMetersSquared,
                IntakeConstants.gearRatio),
            IntakeConstants.Sim.motorGearBox);

    appliedVolts = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.setInputVoltage(appliedVolts);
    intakeMotor.update(0.02);

    inputs.positionRad = intakeMotor.getAngularPositionRotations();
    inputs.velocityRadPerSec = intakeMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = intakeMotor.getCurrentDrawAmps();
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
