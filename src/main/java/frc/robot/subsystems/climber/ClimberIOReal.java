// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

/** The real implementation of the climber. */
public class ClimberIOReal implements ClimberIO {

  private SparkMax climberMotor;
  private RelativeEncoder encoder;

  private DigitalInput limitSwitch;

  /** Creates a new real climber. */
  public ClimberIOReal() {

    climberMotor = new SparkMax(ClimberConstants.Real.motorID, MotorType.kBrushless);
    encoder = climberMotor.getEncoder();
    limitSwitch = new DigitalInput(ClimberConstants.Real.limitSwitchID);

    climberMotor.configure(
        ClimberConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSecond = encoder.getVelocity();
    inputs.appliedVolts = climberMotor.getBusVoltage() * climberMotor.getAppliedOutput();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.hardStop = limitSwitch.get();
  }

  @Override
  public void setTargetHeight(double height) {
    climberMotor.getClosedLoopController().setSetpoint(height, ControlType.kPosition);
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(ClimberConstants.retractedHeight);
  }
}
