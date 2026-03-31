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
import edu.wpi.first.math.geometry.Rotation2d;

/** The real implementation of the intake. */
public class IntakeIOReal implements IntakeIO {

  private SparkMax rollerMotor;
  private RelativeEncoder rollerEncoder;

  private SparkMax pivotMotor;
  private RelativeEncoder pivotEncoder;

  /** Creates a new real intake. */
  public IntakeIOReal() {

    rollerMotor = new SparkMax(IntakeConstants.Real.rollerMotorID, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();
    rollerMotor.configure(
        IntakeConstants.Real.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotor = new SparkMax(IntakeConstants.Real.pivotMotorID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotMotor.configure(
        IntakeConstants.Real.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rollerPositionRotations = rollerEncoder.getPosition();
    inputs.rollerVelocityRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVolts = rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput();
    inputs.rollerCurrentAmps = rollerMotor.getOutputCurrent();

    inputs.pivotPositionRad = pivotEncoder.getPosition();
    inputs.pivotVelocityRadPerSec = pivotEncoder.getVelocity();
    inputs.pivotAppliedVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotMotor.getClosedLoopController().setSetpoint(rot.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setRollerVolts(double volts) {
    rollerMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
