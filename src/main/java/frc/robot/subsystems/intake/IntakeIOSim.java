// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the intake */
public class IntakeIOSim implements IntakeIO {

  private DCMotorSim pivotMotor;
  private PIDController pivotController;
  private Rotation2d pivotTarget;
  private double pivotAppliedVolts;

  private DCMotorSim rollerMotor;
  private double rollerAppliedVolts;

  /** Creates a new sim intake. */
  public IntakeIOSim() {

    pivotMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Sim.pivotGearBox,
                IntakeConstants.Sim.pivotJKgMetersSquared,
                IntakeConstants.pivotGearRatio),
            IntakeConstants.Sim.pivotGearBox);

    pivotController =
        new PIDController(
            IntakeConstants.Sim.pivotP, IntakeConstants.Sim.pivotI, IntakeConstants.Sim.pivotD);

    pivotMotor.setState(IntakeConstants.retractAngle.getRadians(), 0.0);
    pivotTarget = IntakeConstants.retractAngle;
    pivotAppliedVolts = 0.0;

    rollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Sim.rollerGearBox,
                IntakeConstants.Sim.rollerJKgMetersSquared,
                IntakeConstants.rollersGearRatio),
            IntakeConstants.Sim.rollerGearBox);

    rollerAppliedVolts = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs) {

    // This line will run on a real motor controller at 1kHz
    pivotAppliedVolts =
        pivotController.calculate(pivotMotor.getAngularPositionRad(), pivotTarget.getRadians());

    pivotMotor.setInputVoltage(pivotAppliedVolts);
    pivotMotor.update(0.02);

    rollerMotor.setInputVoltage(rollerAppliedVolts);
    rollerMotor.update(0.02);

    inputs.pivotPositionRad = pivotMotor.getAngularPositionRad();
    inputs.pivotVelocityRadPerSec = pivotMotor.getAngularVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = pivotMotor.getCurrentDrawAmps();

    inputs.rollerPositionRotations = rollerMotor.getAngularPositionRotations();
    inputs.rollerVelocityRPM = rollerMotor.getAngularVelocityRPM();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = rollerMotor.getCurrentDrawAmps();
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    pivotTarget = angle;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }
}
