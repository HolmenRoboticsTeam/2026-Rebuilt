// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

  private SparkMax intakeMotor;

  public IntakeIOReal() {
    intakeMotor = new SparkMax(10, MotorType.kBrushed);
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  @Override
  public void setVoltage(double volts) {
    intakeMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
