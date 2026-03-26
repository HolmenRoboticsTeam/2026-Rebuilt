// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

/** The real implementation of the indexer. */
public class IndexerIOReal implements IndexerIO {

  private SparkMax indexerMotor;
  private RelativeEncoder encoder;

  private DigitalInput lineBreak;

  /** Creates a new real indexer. */
  public IndexerIOReal() {

    indexerMotor = new SparkMax(IndexerConstants.Real.motorID, MotorType.kBrushless);
    encoder = indexerMotor.getEncoder();

    indexerMotor.configure(
        IndexerConstants.Real.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    lineBreak = new DigitalInput(IndexerConstants.Real.lineBreakID);
  }

  public void updateInputs(IndexerIOInputs inputs) {

    inputs.positionRotation = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.currentAmps = indexerMotor.getOutputCurrent();
    inputs.hasFuel = !lineBreak.get();
  }

  @Override
  public void setVolts(double volts) {
    indexerMotor.getClosedLoopController().setSetpoint(volts, ControlType.kVoltage);
  }
}
