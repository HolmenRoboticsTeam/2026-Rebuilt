// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class HopperIOReal implements HopperIO {

  private SparkMax hopperMotor;

  public HopperIOReal() {
    hopperMotor = new SparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(HopperIOInputsAutoLogged inputs) {}

  @Override
  public void setVolts(double volts) {
    hopperMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
