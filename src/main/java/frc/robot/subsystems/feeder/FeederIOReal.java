// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

/** The real implementation of the climber. */
public class FeederIOReal implements FeederIO {

  private SparkMax feederMotor;
  private RelativeEncoder encoder;

  private DigitalInput enterLineBreak;
  private DigitalInput exitLineBreak;

  /** Creates a new real climber. */
  public FeederIOReal() {

    feederMotor = new SparkMax(FeederConstants.Real.motorID, MotorType.kBrushless);
    encoder = feederMotor.getEncoder();
    enterLineBreak = new DigitalInput(FeederConstants.Real.enterLineBreakID);
    exitLineBreak = new DigitalInput(FeederConstants.Real.exitLineBreakID);

    feederMotor.configure(
        FeederConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(FeederIOInputs inputs) {

    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = feederMotor.getBusVoltage() * feederMotor.getAppliedOutput();
    inputs.currentAmps = feederMotor.getOutputCurrent();
    inputs.hasEnterFuel = !enterLineBreak.get();
    inputs.hasExitFuel = !exitLineBreak.get();
    inputs.releasingFuel = encoder.getVelocity() > 0.0;
  }

  @Override
  public void setVolts(double volts) {
    feederMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
