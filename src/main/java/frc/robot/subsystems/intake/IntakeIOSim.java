// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** The sim implementation of the intake */
public class IntakeIOSim implements IntakeIO {

  private DCMotorSim rollerMotor;
  private double rollerAppliedVolts;

  private DCMotorSim pivotMotor;
  private PIDController pivotPidController;
  private Rotation2d pivotTargetPosition;

  /** Creates a new sim intake. */
  public IntakeIOSim() {

    rollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Sim.rollerMotorGearBox,
                IntakeConstants.Sim.rollerJKgMetersSquared,
                IntakeConstants.rollerGearRatio),
            IntakeConstants.Sim.rollerMotorGearBox);

    rollerAppliedVolts = 0.0;

    pivotMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.Sim.pivotMotorGearBox,
                IntakeConstants.Sim.pivotJKgMetersSquared,
                IntakeConstants.pivotGearRatio),
            IntakeConstants.Sim.pivotMotorGearBox);
    pivotPidController =
        new PIDController(
            IntakeConstants.Sim.simPivotP,
            IntakeConstants.Sim.simPivotI,
            IntakeConstants.Sim.simPivotD);

    pivotTargetPosition = Rotation2d.kZero;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    rollerMotor.setInputVoltage(rollerAppliedVolts);
    rollerMotor.update(0.02);

    pivotMotor.setInputVoltage(
        pivotPidController.calculate(
            pivotMotor.getAngularPosition().in(Radians), pivotTargetPosition.getRadians()));
    pivotMotor.update(0.02);

    inputs.rollerAppliedVolts = rollerMotor.getInputVoltage();
    inputs.rollerCurrentAmps = rollerMotor.getCurrentDrawAmps();
    inputs.rollerPositionRotations = rollerMotor.getAngularPositionRotations();
    inputs.rollerVelocityRPM = rollerMotor.getAngularVelocityRPM();

    inputs.pivotAppliedVolts = pivotMotor.getInputVoltage();
    inputs.pivotCurrentAmps = pivotMotor.getCurrentDrawAmps();
    inputs.pivotPositionRad = pivotMotor.getAngularPositionRad();
    inputs.pivotVelocityRadPerSec = pivotMotor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setPivotPosition(Rotation2d rot) {}

  @Override
  public void setRollerVolts(double volts) {
    rollerAppliedVolts = volts;
  }
}
