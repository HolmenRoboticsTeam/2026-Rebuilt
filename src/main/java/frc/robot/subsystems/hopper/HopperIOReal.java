// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/** The real implementation of the hopper. */
public class HopperIOReal implements HopperIO {

  private SparkMax hopperMotor;
  private RelativeEncoder encoder;

  /** Creates a new real feeder. */
  public HopperIOReal() {

    hopperMotor = new SparkMax(HopperConstants.Real.motorID, MotorType.kBrushless);
    encoder = hopperMotor.getEncoder();

    hopperMotor.configure(
        HopperConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(HopperIOInputs inputs) {

    inputs.positionRotations = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = hopperMotor.getBusVoltage() * hopperMotor.getAppliedOutput();
    inputs.currentAmps = hopperMotor.getOutputCurrent();
  }

  public void setVoltage(double volts) {
    hopperMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
