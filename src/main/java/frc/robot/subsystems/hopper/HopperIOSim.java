// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the hopper. */
public class HopperIOSim implements HopperIO {

  private DCMotorSim hopperMotor;

  private double appliedVolts;

  /** Creates a new sim hopper. */
  public HopperIOSim() {

    hopperMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HopperConstants.Sim.motorGearBox,
                HopperConstants.Sim.JKgMetersSquared,
                HopperConstants.gearRatio),
            HopperConstants.Sim.motorGearBox);

    appliedVolts = 0.0;
  }

  public void updateInputs(HopperIOInputs inputs) {
    hopperMotor.setInputVoltage(appliedVolts);
    hopperMotor.update(0.02);

    inputs.positionRotation = hopperMotor.getAngularPositionRotations();
    inputs.velocityRPM = hopperMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = hopperMotor.getCurrentDrawAmps();
    inputs.hasFuel = true;
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = volts;
  }
}
