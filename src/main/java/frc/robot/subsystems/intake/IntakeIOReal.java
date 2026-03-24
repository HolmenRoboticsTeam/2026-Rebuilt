// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** The real implementation of the intake. */
public class IntakeIOReal implements IntakeIO {

  private SparkMax intakeMotor;
  private RelativeEncoder encoder;

  /** Creates a new real intake. */
  public IntakeIOReal() {

    intakeMotor = new SparkMax(IntakeConstants.Real.motorID, MotorType.kBrushless);
    encoder = intakeMotor.getEncoder();

    intakeMotor.configure(
        IntakeConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
    inputs.isRunning = encoder.getVelocity() > 0.0;
  }

  @Override
  public void setVolts(double volts) {
    intakeMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
