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

  private SparkMax hopperAcrossMotor;
  private SparkMax hopperTowardsMotor;
  private RelativeEncoder encoder;

  private DigitalInput linebreak;

  /** Creates a new real hopper. */
  public HopperIOReal() {

    hopperAcrossMotor = new SparkMax(HopperConstants.Real.acrossMotorID, MotorType.kBrushless);
    hopperTowardsMotor = new SparkMax(HopperConstants.Real.towardsMotorID, MotorType.kBrushless);

    encoder = hopperAcrossMotor.getEncoder();

    hopperAcrossMotor.configure(
        HopperConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    linebreak = new DigitalInput(HopperConstants.Real.lineBreakID);
    hopperTowardsMotor.configure(
        HopperConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void updateInputs(HopperIOInputs inputs) {

    inputs.positionRotation = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = hopperAcrossMotor.getBusVoltage() * hopperAcrossMotor.getAppliedOutput();
    inputs.currentAmps = hopperAcrossMotor.getOutputCurrent();
    inputs.hasFuel = !linebreak.get();
  }

  @Override
  public void setVolts(double acrossVolts, double towardsVolts) {
    hopperAcrossMotor.getClosedLoopController().setSetpoint(acrossVolts, ControlType.kVoltage);
    hopperTowardsMotor.getClosedLoopController().setSetpoint(towardsVolts, ControlType.kVoltage);
  }
}
