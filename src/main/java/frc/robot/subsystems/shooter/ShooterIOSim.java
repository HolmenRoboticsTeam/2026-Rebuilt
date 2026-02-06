// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {

  private DCMotorSim shooterMotor;

  public ShooterIOSim() {
    shooterMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.025, 1.0), DCMotor.getCIM(1));
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  @Override
  public void setVoltage(double volts) {
    shooterMotor.setInputVoltage(volts);
  }
}
