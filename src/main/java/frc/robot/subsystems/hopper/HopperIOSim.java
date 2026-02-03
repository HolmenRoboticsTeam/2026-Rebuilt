// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class HopperIOSim implements HopperIO {

  private DCMotorSim hopperMotor;

  private double appliedVolts;

  public HopperIOSim() {
    hopperMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, 1.0), DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(HopperIOInputsAutoLogged inputs) {
    hopperMotor.setInputVoltage(appliedVolts);
    hopperMotor.update(0.02);
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = volts;
  }
}
