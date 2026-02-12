// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class IndexIOReal implements IndexIO {

  private SparkMax indexMotor;

  public IndexIOReal() {
    indexMotor = new SparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IndexIOInputsAutoLogged inputs) {}
  @Override
  public void setVoltage(double volts) {
    indexMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
