// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LightCommands;
import frc.robot.commands.StateLoggingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.SwitchBoard;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  private Intake intake;
  private Hopper hopper;
  private Feeder feeder;
  private Turret turret;

  // Auto
  private final LoggedDashboardChooser<Boolean> autoSelectorType;
  private final LoggedDashboardChooser<Command> hardAutoChooser;
  private final Field2d autoField = new Field2d();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final SwitchBoard switchBoard = new SwitchBoard(1, 2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                (p, t, sd) -> drive.addVisionPoseMeasurement(p, t, sd),
                new VisionIOLimelight("limelight-left", () -> drive.getRotation()),
                new VisionIOLimelight("limelight-right", () -> drive.getRotation()));
        intake = new Intake(new IntakeIOReal());
        hopper = new Hopper(new HopperIOReal());
        feeder = new Feeder(new FeederIOReal(), () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIOReal(),
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                (p, t, sd) -> drive.addVisionPoseMeasurement(p, t, sd),
                // new VisionIOPhotonVisionSim(
                //     "Camera0",
                //     VisionConstants.robotToCamera0,
                //     drive.getPose), // TODO: set these Transforms
                // new VisionIOPhotonVisionSim(
                //     "Camera1", VisionConstants.robotToCamera1, drive.getPose));
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new IntakeIOSim());
        hopper = new Hopper(new HopperIOSim());
        feeder = new Feeder(new FeederIOSim(), () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIOSim(),
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> {});

        // Register a robot for collision with fuel
        FuelSim.getInstance()
            .registerRobot(
                Distance.ofRelativeUnits(29.0, Inches).in(Meters), // from left to right
                Distance.ofRelativeUnits(26.0, Inches).in(Meters), // from front to back
                Distance.ofRelativeUnits(6.0, Inches).in(Meters), // from floor to top of bumpers
                () -> drive.getPose(), // Supplier<Pose2d> of robot pose
                () -> drive.getChassisSpeeds()); // Supplier<ChassisSpeeds> of field-centric chassis
        // speeds

        FuelSim.getInstance()
            .registerIntake(
                Distance.ofRelativeUnits(13.0, Inches).in(Meters),
                Distance.ofRelativeUnits(16.0, Inches).in(Meters),
                Distance.ofRelativeUnits(-8.0, Inches).in(Meters),
                Distance.ofRelativeUnits(8.0, Inches)
                    .in(Meters), // robot-centric coordinates for bounding box
                () -> {
                  return intake.isRunning();
                }, // (optional) BooleanSupplier for whether the intake should be active at a given
                // moment
                () -> {}); // (optional) Runnable called whenever a fuel is in-took
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                (p, t, sd) -> drive.addVisionPoseMeasurement(p, t, sd),
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        feeder = new Feeder(new FeederIO() {}, () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIO() {},
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> {});
        break;
    }

    // Auto setup
    List<Pair<String, Command>> commands =
        Constants.getNamedCommand(drive, vision, intake, hopper, feeder, turret);
    for (Pair<String, Command> command : commands) {
      NamedCommands.registerCommand(command.getFirst(), command.getSecond());
    }
    autoSelectorType = new LoggedDashboardChooser<>("Auto Selector Type");
    autoSelectorType.addDefaultOption("Use Button Box", true);
    autoSelectorType.addOption("Use Dashboard Chooser", false);
    hardAutoChooser = new LoggedDashboardChooser<>("Choose Auto", AutoBuilder.buildAutoChooser());

    // SysId routines
    hardAutoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    hardAutoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    hardAutoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    hardAutoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    hardAutoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    hardAutoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> controller.getRightTriggerAxis(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightY(),
            () -> -controller.getRightX()));

    feeder.setDefaultCommand(feeder.autoFeed());
    turret.setDefaultCommand(turret.fullFieldAim());

    // Auto Field
    SmartDashboard.putData("AutoField", autoField);
    CommandScheduler.getInstance()
        .schedule(
            AutoCommands.displayAutoField(autoField, () -> getAutoName(), () -> drive.getPose()));

    // Configure the button bindings
    configureButtonBindings();

    // Schedule start-up commands.
    CommandScheduler.getInstance()
        .schedule(
            PathfindingCommand.warmupCommand().withName("Pathplanner_Warmup"),
            StateLoggingCommands.logMechanisms(intake, hopper, feeder, turret),
            StateLoggingCommands.updateDashboard(),
            StateLoggingCommands.rumbleOnShiftChange(controller),
            LightCommands.controlLights(
                () ->
                    MathUtil.clamp(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0, 25.0)
                        / 25.0,
                Color.kRed,
                Color.kYellow,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack),
            // Call these here, so that the controls is ready
            intake.start().beforeStarting(Commands.waitSeconds(5.0)),
            hopper.start().beforeStarting(Commands.waitSeconds(5.0)),
            turret.zeroRotationOffEncoder().beforeStarting(Commands.waitSeconds(5.0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // ######################################## #################
    // ######################################## DRIVER CONTROLLER
    // ######################################## #################

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to ZERO when Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(() -> drive.resetGyro()).ignoringDisable(true).withName("Zero Gyro"));

    // Testing controls
    controller.a().onTrue(intake.start()).onFalse(intake.stop());
    controller.b().onTrue(hopper.start()).onFalse(hopper.stop());
    controller.y().onTrue(feeder.start()).onFalse(feeder.stop());

    // ######################################## ############
    // ######################################## BUTTON BOARD
    // ######################################## ############

    // #################### ROW ONE ####################

    // Corralling fuel to the depot (Left side)
    switchBoard
        .get(1, 1)
        .whileTrue(
            Commands.either(
                Commands.either( // Alliance Side
                    // This path will be selected when the robot is in our alliance zone or our
                    // bump/trench
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Depot Extra Short", false),
                    // This path will be selected when the robot is on the close side of the neutral
                    // zone.
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Depot Short", false),
                    () ->
                        FieldConstants.allianceFlip(FieldConstants.kHomeAllianceZone)
                                .contains(drive.getPose().getTranslation())
                            || FieldConstants.allianceFlip(FieldConstants.kAllianceTrenchBumpZone)
                                .contains(drive.getPose().getTranslation())),
                Commands.either( // Opposing Side
                    // This path will be selected when the robot is in the opposing alliance zone or
                    // the opposing alliance bump/trench
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Depot Long", false),
                    // This path will be selected when the robot is on the far side of the neutral
                    // zone.
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Depot Mid", false),
                    () ->
                        FieldConstants.allianceFlip(FieldConstants.kOpposingAllianceZone)
                                .contains(drive.getPose().getTranslation())
                            || FieldConstants.allianceFlip(FieldConstants.kOpposingTrenchBumpZone)
                                .contains(drive.getPose().getTranslation())),
                () ->
                    FieldConstants.allianceFlip(FieldConstants.kAllianceHalf)
                        .contains(drive.getPose().getTranslation())));

    // Sweeping behind the hub
    switchBoard
        .get(1, 2)
        .whileTrue(
            AutoDriveCommands.driveToPoseThenPath(drive, "Right To Left Back Of Hub", false));

    // Sweeping behind the hub
    switchBoard
        .get(1, 3)
        .whileTrue(
            AutoDriveCommands.driveToPoseThenPath(drive, "Left To Right Back Of Hub", false));

    // Corralling fuel into the outpost (Right side)
    switchBoard
        .get(1, 4)
        .whileTrue(
            Commands.either(
                Commands.either( // Alliance Side
                    // This path will be selected when the robot is in our alliance zone or our
                    // bump/trench
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Outpost Extra Short", false),
                    // This path will be selected when the robot is on the close side of the neutral
                    // zone.
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Outpost Short", false),
                    () ->
                        FieldConstants.allianceFlip(FieldConstants.kHomeAllianceZone)
                                .contains(drive.getPose().getTranslation())
                            || FieldConstants.allianceFlip(FieldConstants.kAllianceTrenchBumpZone)
                                .contains(drive.getPose().getTranslation())),
                Commands.either( // Opposing Side
                    // This path will be selected when the robot is in the opposing alliance zone or
                    // the opposing alliance bump/trench
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Outpost Long", false),
                    // This path will be selected when the robot is on the far side of the neutral
                    // zone.
                    AutoDriveCommands.driveToPoseThenPath(drive, "Feed Outpost Mid", false),
                    () ->
                        FieldConstants.allianceFlip(FieldConstants.kOpposingAllianceZone)
                                .contains(drive.getPose().getTranslation())
                            || FieldConstants.allianceFlip(FieldConstants.kOpposingTrenchBumpZone)
                                .contains(drive.getPose().getTranslation())),
                () ->
                    FieldConstants.allianceFlip(FieldConstants.kAllianceHalf)
                        .contains(drive.getPose().getTranslation())));

    // #################### ROW TWO ####################
    switchBoard
        .get(2, 2)
        .whileTrue(hopper.reverse())
        .onFalse(
            Commands.either(
                hopper.start(), hopper.stop(), () -> switchBoard.get(3, 2).getAsBoolean()));

    switchBoard.get(2, 3).onTrue(turret.maxFlyWheel());

    // #################### ROW THREE ####################
    switchBoard.get(3, 1).onTrue(intake.start()).onFalse(intake.reverse());

    switchBoard.get(3, 2).onTrue(hopper.start()).onFalse(hopper.stop());

    switchBoard.get(3, 3).onTrue(intake.extend()).onFalse(intake.retract());

    switchBoard
        .get(3, 4)
        .whileTrue(
            Commands.either(
                turret.lockRotationToZero(),
                turret.calibrate(),
                () -> DriverStation.isFMSAttached()))
        .whileFalse(turret.fullFieldAim());

    // Shift Overriding

    switchBoard
        .axisLessThan(3, -0.5)
        .onTrue(
            Commands.runOnce(
                    () -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(Alliance.Blue)))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty()))
                .ignoringDisable(true));
    switchBoard
        .axisLessThan(3, 0.5)
        .onTrue(
            Commands.runOnce(
                    () -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(Alliance.Red)))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty()))
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (autoSelectorType.get()) {
      return new PathPlannerAuto(getAutoName());
    } else {
      return hardAutoChooser.get();
    }
  }

  private String getAutoName() {
    String[] autonomousData =
        new String[] {
          switchBoard.getRawAxis(0) < -0.5
              ? "Left"
              : (switchBoard.getRawAxis(0) > 0.5 ? "Right" : "Hub"),
          switchBoard.getRawAxis(1) < -0.5
              ? "Left"
              : (switchBoard.getRawAxis(1) > 0.5 ? "Right" : "Mid"),
          switchBoard.getRawAxis(2) < -0.5
              ? "Left"
              : (switchBoard.getRawAxis(2) > 0.5 ? "Right" : "Mid")
        };

    return autonomousData[0] + "-" + autonomousData[1] + "-" + autonomousData[2];
  }

  public void enabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(4));
  }

  public void disabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(1));
  }

  public void endAuto() {
    vision.recordLastSecond(30.0);
  }

  public void endMatch() {
    vision.recordLastSecond(145.0);
  }
}
