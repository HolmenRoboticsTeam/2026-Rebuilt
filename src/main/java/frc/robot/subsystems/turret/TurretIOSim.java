// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;

/** The sim implementation of the turret. */
public class TurretIOSim implements TurretIO {

  private DCMotorSim rotationMotorSim;
  private PIDController rotationController;
  private Rotation2d targetRotation = new Rotation2d();
  private double rotationAppliedVolts;
  private Supplier<Double> getRotationOffset = () -> Constants.isBlueAlliance.get() ? 0.0 : Math.PI;

  private DCMotorSim angleMotorSim;
  private PIDController angleController;
  private Rotation2d targetAngle = new Rotation2d();
  private double angleAppliedVolts;

  private FlywheelSim flyWheelSim;
  private PIDController flyWheelController;
  private double targetRPM;
  private double flyWheelAppliedVolts;

  /** Creates a new sim turret. */
  public TurretIOSim() {

    // Rotation Motor setup

    rotationMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.Sim.rotationMotorGearBox,
                TurretConstants.Sim.rotationJKgMetersSquared,
                TurretConstants.rotationGearing),
            TurretConstants.Sim.rotationMotorGearBox);
    rotationController =
        new PIDController(
            TurretConstants.Sim.rotationP,
            TurretConstants.Sim.rotationI,
            TurretConstants.Sim.rotationD);

    // Angle Motor setup

    angleMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.Sim.angleMotorGearBox,
                TurretConstants.Sim.angleJKgMetersSquared,
                TurretConstants.angleGearing),
            TurretConstants.Sim.angleMotorGearBox);
    angleController =
        new PIDController(
            TurretConstants.Sim.angleP, TurretConstants.Sim.angleI, TurretConstants.Sim.angleD);

    // Fly Wheel Motor setup

    flyWheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                TurretConstants.Sim.flyWheelMotorGearBox,
                TurretConstants.Sim.flyWheelJKgMetersSquared,
                TurretConstants.flyWheelGearing),
            TurretConstants.Sim.flyWheelMotorGearBox);
    flyWheelController =
        new PIDController(
            TurretConstants.Sim.flyWheelP,
            TurretConstants.Sim.flyWheelI,
            TurretConstants.Sim.flyWheelD);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // All three of these lines run on the sparkMax for real (at 100hz?)
    rotationAppliedVolts =
        rotationController.calculate(
            rotationMotorSim.getAngularPositionRad(), targetRotation.getRadians());

    angleAppliedVolts =
        angleController.calculate(angleMotorSim.getAngularPositionRad(), targetAngle.getRadians());

    flyWheelAppliedVolts =
        flyWheelController.calculate(
            flyWheelSim.getAngularVelocityRadPerSec(),
            Units.rotationsPerMinuteToRadiansPerSecond(targetRPM));

    rotationMotorSim.setInputVoltage(rotationAppliedVolts);
    rotationMotorSim.update(0.02);

    angleMotorSim.setInputVoltage(angleAppliedVolts);
    angleMotorSim.update(0.02);

    flyWheelSim.setInputVoltage(flyWheelAppliedVolts);
    flyWheelSim.update(0.02);

    inputs.rotationPositionRad = rotationMotorSim.getAngularPositionRad();
    inputs.rotationVelocityRadPerSec = rotationMotorSim.getAngularVelocityRadPerSec();
    inputs.rotationAppliedVolts = rotationAppliedVolts;
    inputs.rotationCurrentAmps = rotationMotorSim.getCurrentDrawAmps();
    inputs.rotationIsAtTarget =
        MathUtil.isNear(
            targetRotation.getRadians(),
            rotationMotorSim.getAngularPositionRad(),
            TurretConstants.rotationTolerance);

    inputs.anglePositionRad = angleMotorSim.getAngularPositionRad();
    inputs.angleVelocityRadPerSec = angleMotorSim.getAngularVelocityRadPerSec();
    inputs.angleAppliedVolts = angleAppliedVolts;
    inputs.angleCurrentAmps = angleMotorSim.getCurrentDrawAmps();
    inputs.angleIsAtTarget =
        MathUtil.isNear(
            targetAngle.getRadians(),
            angleMotorSim.getAngularPositionRad(),
            TurretConstants.angleTolerance);

    inputs.flyWheelPositionRotations =
        inputs.flyWheelPositionRotations + (flyWheelSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.flyWheelVelocityRPM = flyWheelSim.getAngularVelocityRPM();
    inputs.flyWheelAppliedVolts = flyWheelAppliedVolts;
    inputs.flyWheelCurrentAmps = flyWheelSim.getCurrentDrawAmps();
    inputs.flyWheelIsTarget =
        MathUtil.isNear(
            targetRPM, flyWheelSim.getAngularVelocityRPM(), TurretConstants.flyWheelTolerance);
  }

  @Override
  public void setTargetRotation(Rotation2d rot) {
    targetRotation = rot.plus(Rotation2d.fromRadians(getRotationOffset.get()));
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    targetAngle = angle;
  }

  @Override
  public void setFlyWheelRPM(double RPM) {
    targetRPM = RPM;
  }

  @Override
  public void shootFuel(Rotation2d robotRotation) {

    Rotation2d trueRot =
        Rotation2d.fromRadians(rotationMotorSim.getAngularPositionRad()).plus(robotRotation);
    double angle = angleMotorSim.getAngularPositionRad();
    double rpm = flyWheelSim.getAngularVelocityRPM();

    // Eat some speed
    flyWheelSim.setAngularVelocity(
        flyWheelSim.getAngularVelocityRadPerSec() * TurretConstants.Sim.percentSpeedKept);

    FuelSim.getInstance()
        .launchFuel(
            LinearVelocity.ofRelativeUnits(
                TurretConstants.Sim.RPMToVelocityFactor * rpm, MetersPerSecond),
            Angle.ofRelativeUnits(angle, Radian),
            trueRot.getMeasure(),
            TurretConstants.Sim.turretHeight);
  }
}
