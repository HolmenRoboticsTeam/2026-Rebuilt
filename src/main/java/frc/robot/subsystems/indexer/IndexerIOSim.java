// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the indexer. */
public class IndexerIOSim implements IndexerIO {

  private DCMotorSim indexerMotor;

  private double appliedVolts;

  /** Creates a new sim indexer. */
  public IndexerIOSim() {

    indexerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IndexerConstants.Sim.motorGearBox,
                IndexerConstants.Sim.JKgMetersSquared,
                IndexerConstants.gearRatio),
            IndexerConstants.Sim.motorGearBox);

    appliedVolts = 0.0;
  }

  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotor.setInputVoltage(appliedVolts);
    indexerMotor.update(0.02);

    inputs.positionRotation = indexerMotor.getAngularPositionRotations();
    inputs.velocityRPM = indexerMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = indexerMotor.getCurrentDrawAmps();
    inputs.hasFuel = true;
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = volts;
  }
}
