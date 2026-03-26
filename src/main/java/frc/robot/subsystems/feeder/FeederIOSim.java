// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the feeder. */
public class FeederIOSim implements FeederIO {

  private DCMotorSim feederMotor;

  private double appliedVolts;

  /** Creates a new sim feeder. */
  public FeederIOSim() {

    feederMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FeederConstants.Sim.motorGearBox,
                FeederConstants.Sim.JKgMetersSquared,
                FeederConstants.gearRatio),
            FeederConstants.Sim.motorGearBox);

    appliedVolts = 0.0;
  }

  public void updateInputs(FeederIOInputs inputs) {
    feederMotor.setInputVoltage(appliedVolts);
    feederMotor.update(0.02);

    inputs.positionRotations = feederMotor.getAngularPositionRotations();
    inputs.velocityRPM = feederMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = feederMotor.getCurrentDrawAmps();
    inputs.hasEnterFuel = true; // Always has fuel
    inputs.hasExitFuel = true;
    inputs.releasingFuel = appliedVolts > 0.0;
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = volts;
  }
}
