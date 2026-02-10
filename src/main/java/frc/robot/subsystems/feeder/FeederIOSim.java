// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

  private DCMotorSim feederMotor;

  private double appliedVolts;

  public FeederIOSim() {
    feederMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 1.0), DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    feederMotor.setInputVoltage(appliedVolts);
    feederMotor.update(0.02);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
