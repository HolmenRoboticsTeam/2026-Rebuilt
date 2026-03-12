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
import edu.wpi.first.wpilibj.DigitalInput;

/** The real implementation of the hopper. */
public class HopperIOReal implements HopperIO {

  private SparkMax hopperMotor;
  private RelativeEncoder encoder;

  private DigitalInput lineBreak;

  /** Creates a new real hopper. */
  public HopperIOReal() {

    hopperMotor = new SparkMax(HopperConstants.Real.motorID, MotorType.kBrushless);
    encoder = hopperMotor.getEncoder();

    hopperMotor.configure(
        HopperConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    lineBreak = new DigitalInput(HopperConstants.Real.lineBreakID);
  }

  public void updateInputs(HopperIOInputs inputs) {

    inputs.positionRotation = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = hopperMotor.getBusVoltage() * hopperMotor.getAppliedOutput();
    inputs.currentAmps = hopperMotor.getOutputCurrent();
    inputs.hasFuel = lineBreak.get();
  }

  @Override
  public void setVolts(double volts) {
    hopperMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
