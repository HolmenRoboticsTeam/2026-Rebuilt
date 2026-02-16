// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the climber. */
public class ClimberIOSim implements ClimberIO {

  private DCMotorSim climberMotor;

  private double appliedVolts;

  /** Creates a new sim climber. */
  public ClimberIOSim() {

    climberMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.Sim.motorGearBox,
                ClimberConstants.Sim.JKgMetersSquared,
                ClimberConstants.motorToLengthRatio),
            ClimberConstants.Sim.motorGearBox);

    appliedVolts = 0.0;
  }

  public void updateInputs(ClimberIOInputs inputs) {
    climberMotor.setInputVoltage(appliedVolts);
    climberMotor.update(0.02);

    inputs.positionMeters =
        climberMotor.getAngularPositionRotations() * ClimberConstants.motorToLengthRatio;
    inputs.velocityMetersPerSecond =
        climberMotor.getAngularVelocityRPM() * ClimberConstants.motorToLengthRatio / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = climberMotor.getCurrentDrawAmps();
    inputs.hardStop =
        climberMotor.getAngularPositionRotations() <= ClimberConstants.retractedHeight;
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
