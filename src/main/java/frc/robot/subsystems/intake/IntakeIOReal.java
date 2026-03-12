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

  private SparkMax pivotMotor;
  private RelativeEncoder pivotEncoder;

  private SparkMax rollerMotor;
  private RelativeEncoder rollerEncoder;

  /** Creates a new real intake. */
  public IntakeIOReal() {

    pivotMotor = new SparkMax(IntakeConstants.Real.pivotMotorID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();

    pivotMotor.configure(
        IntakeConstants.Real.pivotMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rollerMotor = new SparkMax(IntakeConstants.Real.rollerMotorID, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();

    rollerMotor.configure(
        IntakeConstants.Real.rollerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.pivotPositionRad = pivotEncoder.getPosition();
    inputs.pivotVelocityRadPerSec = pivotEncoder.getVelocity();
    inputs.pivotAppliedVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();

    inputs.rollerPositionRotations = rollerEncoder.getPosition();
    inputs.rollerVelocityRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVolts = rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput();
    inputs.rollerCurrentAmps = rollerMotor.getOutputCurrent();
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    pivotMotor.getClosedLoopController().setSetpoint(angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
