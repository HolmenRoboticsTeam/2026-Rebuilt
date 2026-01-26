// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

  private SparkMax intakeMotor;
  private RelativeEncoder intakeEncoder;

  public IntakeIOReal() {
    intakeMotor = new SparkMax(10, MotorType.kBrushed);
    intakeEncoder = intakeMotor.getEncoder();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
