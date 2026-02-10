// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class FeederIOReal implements FeederIO {

  private SparkMax feederMotor;

  public FeederIOReal() {
    feederMotor = new SparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {}

  @Override
  public void setVoltage(double volts) {
    feederMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
