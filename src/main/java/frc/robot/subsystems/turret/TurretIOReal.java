// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** The real implementation of the turret. */
public class TurretIOReal implements TurretIO {

  private SparkFlex rotationMotor;
  private RelativeEncoder rotationEncoder;
  private boolean isLocked;

  private SparkMax angleMotor;
  private RelativeEncoder angleEncoder;

  private SparkMax flyWheelMotorLeft;
  private SparkMax flyWheelMotorRight;
  private RelativeEncoder flyWheelEncoderLeft;

  /** Creates a new real turret. */
  public TurretIOReal() {

    // Rotation Motor setup

    rotationMotor = new SparkFlex(TurretConstants.Real.rotationMotorID, MotorType.kBrushless);
    rotationEncoder = rotationMotor.getEncoder();
    rotationEncoder.setPosition(rotationMotor.getAbsoluteEncoder().getPosition());

    rotationMotor.configure(
        TurretConstants.Real.rotationMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    isLocked = false;

    // Angle Motor setup

    angleMotor = new SparkMax(TurretConstants.Real.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();

    angleMotor.configure(
        TurretConstants.Real.angleMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Fly Wheel Motor setup

    flyWheelMotorLeft =
        new SparkMax(TurretConstants.Real.leftFlyWheelMotorID, MotorType.kBrushless);
    flyWheelMotorRight =
        new SparkMax(TurretConstants.Real.rightFlyWheelMotorID, MotorType.kBrushless);
    flyWheelEncoderLeft = flyWheelMotorLeft.getEncoder();

    flyWheelMotorLeft.configure(
        TurretConstants.Real.leftFlyWheelMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flyWheelMotorRight.configure(
        TurretConstants.Real.rightFlyWheelMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.rotationPositionRad = rotationEncoder.getPosition();
    inputs.rotationVelocityRadPerSec = rotationEncoder.getVelocity();
    inputs.rotationAppliedVolts = rotationMotor.getBusVoltage() * rotationMotor.getAppliedOutput();
    inputs.rotationCurrentAmps = rotationMotor.getOutputCurrent();
    inputs.rotationIsAtTarget =
        MathUtil.isNear(
            rotationMotor.getClosedLoopController().getSetpoint(),
            rotationMotor.getEncoder().getPosition(),
            TurretConstants.rotationTolerance);

    inputs.anglePositionRad = angleEncoder.getPosition();
    inputs.angleVelocityRadPerSec = angleEncoder.getVelocity();
    inputs.angleAppliedVolts = angleMotor.getBusVoltage() * angleMotor.getAppliedOutput();
    inputs.angleCurrentAmps = angleMotor.getOutputCurrent();
    inputs.angleIsAtTarget =
        MathUtil.isNear(
            angleMotor.getClosedLoopController().getSetpoint(),
            angleMotor.getEncoder().getPosition(),
            TurretConstants.angleTolerance);

    inputs.flyWheelPositionRotations = flyWheelEncoderLeft.getPosition();
    inputs.flyWheelVelocityRPM = flyWheelEncoderLeft.getVelocity();
    inputs.flyWheelAppliedVolts =
        flyWheelMotorLeft.getBusVoltage() * flyWheelMotorLeft.getAppliedOutput();
    inputs.flyWheelCurrentAmps = flyWheelMotorLeft.getOutputCurrent();
    inputs.flyWheelIsTarget =
        MathUtil.isNear(
            flyWheelMotorLeft.getClosedLoopController().getSetpoint(),
            flyWheelMotorLeft.getEncoder().getVelocity(),
            TurretConstants.flyWheelTolerance);

    // inputs.isTurretAtTarget =
    //     inputs.rotationIsAtTarget && inputs.angleIsAtTarget && inputs.flyWheelIsTarget;

    // This drops the flywheel requirement because the PID for the flyWheel does not reach target,
    // but still hits the shots.
    inputs.isTurretAtTarget = inputs.rotationIsAtTarget && inputs.angleIsAtTarget;
  }

  @Override
  public void setTargetRotation(Rotation2d rot) {
    // If turret is locked, then take no new action.
    if (isLocked) return;

    rotationMotor.getClosedLoopController().setSetpoint(rot.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    angleMotor.getClosedLoopController().setSetpoint(angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setFlyWheelRPM(double RPM) {
    flyWheelMotorLeft.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
  }

  @Override
  public void lockRotation(boolean lockRotation) {
    // Rotation is locked, so updated the value and return;
    if (isLocked) {
      isLocked = lockRotation;
      return;
    }

    // Rotation is not locked. First, set the rotation target to zero, then lock the rotation motor.
    setTargetRotation(new Rotation2d());
    isLocked = lockRotation;
  }

  @Override
  public void zeroRotationOffEncoder() {
    rotationEncoder.setPosition(rotationMotor.getAbsoluteEncoder().getPosition());
  }
}
