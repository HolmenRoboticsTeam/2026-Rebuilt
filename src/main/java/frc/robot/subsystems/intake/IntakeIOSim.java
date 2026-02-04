// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {

  private DCMotorSim intakeMotor;

  public IntakeIOSim() {
    intakeMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.025, 1.0), DCMotor.getCIM(1));
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setInputVoltage(volts);
  }
}
