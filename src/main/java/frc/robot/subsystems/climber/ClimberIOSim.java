// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the climber. */
public class ClimberIOSim implements ClimberIO {

  private DCMotorSim climberMotor;

  private PIDController pidController;
  private double targetHeight;
  private double appliedVolts;

  /** Creates a new sim climber. */
  public ClimberIOSim() {

    climberMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.Sim.motorGearBox,
                ClimberConstants.Sim.JKgMetersSquared,
                ClimberConstants.motorToLengthRatio),
            ClimberConstants.Sim.motorGearBox);

    pidController =
        new PIDController(
            ClimberConstants.Sim.climberP,
            ClimberConstants.Sim.climberI,
            ClimberConstants.Sim.climberD);

    resetPosition();
    targetHeight = climberMotor.getAngularPositionRotations() * ClimberConstants.motorToLengthRatio;
    appliedVolts = 0.0;
  }

  public void updateInputs(ClimberIOInputs inputs) {

    appliedVolts =
        pidController.calculate(
            climberMotor.getAngularPositionRotations() * ClimberConstants.motorToLengthRatio,
            targetHeight);

    climberMotor.setInputVoltage(appliedVolts);
    climberMotor.update(0.02);

    climberMotor.setAngle(
        MathUtil.clamp(
            climberMotor.getAngularPositionRad(),
            2.0 * Math.PI * ClimberConstants.retractedHeight / ClimberConstants.motorToLengthRatio
                - 1.0,
            2.0 * Math.PI * ClimberConstants.extendHeight / ClimberConstants.motorToLengthRatio
                + 1.0));

    inputs.positionMeters =
        climberMotor.getAngularPositionRotations() * ClimberConstants.motorToLengthRatio;
    inputs.velocityMetersPerSecond =
        climberMotor.getAngularVelocityRPM() * ClimberConstants.motorToLengthRatio / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = climberMotor.getCurrentDrawAmps();
    inputs.hardStop =
        climberMotor.getAngularPositionRotations() * ClimberConstants.motorToLengthRatio
            <= ClimberConstants.retractedHeight;
  }

  @Override
  public void setTargetHeight(double height) {
    targetHeight = height;
  }

  @Override
  public void resetPosition() {
    climberMotor.setAngle(
        2.0 * Math.PI * ClimberConstants.retractedHeight / ClimberConstants.motorToLengthRatio);
    climberMotor.setAngularVelocity(0.0);
  }
}
