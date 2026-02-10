// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IndexIOSim implements IndexIO {

  private DCMotorSim indexMotor;

  private double appliedVolts;

  public IndexIOSim() {
    indexMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 1.0), DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(IndexIOInputsAutoLogged inputs) {
    indexMotor.setInputVoltage(appliedVolts);
    indexMotor.update(0.02);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
