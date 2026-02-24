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
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoCommandBuilder;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateLoggingCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.FuelSim;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private final Intake intake;
  private final Hopper hopper;
  private final Indexer indexer;
  private final Feeder feeder;
  private final Turret turret;

  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController buttonBox = new XboxController(1);

  // Left side
  private final JoystickButton reverseFeeder = new JoystickButton(buttonBox, 6);
  private final JoystickButton climbLeftSide = new JoystickButton(buttonBox, 5);
  private final JoystickButton moveToLeftTrench = new JoystickButton(buttonBox, 4);

  // Center
  private final JoystickButton feedOutpost = new JoystickButton(buttonBox, 7);

  // Right Side
  private final JoystickButton unused = new JoystickButton(buttonBox, 3);
  private final JoystickButton climbRightSide = new JoystickButton(buttonBox, 2);
  private final JoystickButton moveToRightTrench = new JoystickButton(buttonBox, 1);

  // Auto
  private final AutoCommandBuilder autoCommandBuilder;
  private final LoggedDashboardChooser<Boolean> useDynamicAuto;
  private final LoggedDashboardChooser<Command> hardAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIONavX(),
        //         new ModuleIOSpark(0),
        //         new ModuleIOSpark(1),
        //         new ModuleIOSpark(2),
        //         new ModuleIOSpark(3));
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-three", drive::getRotation),
                new VisionIOLimelight("Camera1", drive::getRotation));
        // intake = new Intake(new IntakeIOReal());
        // hopper = new Hopper(new HopperIOReal());
        // indexer = new Indexer(new IndexerIOReal());
        // feeder = new Feeder(new FeederIOReal());
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        indexer = new Indexer(new IndexerIO() {});
        feeder = new Feeder(new FeederIO() {});
        turret =
            new Turret(
                new TurretIOReal(),
                drive::getPose,
                drive::getChassisSpeeds,
                feeder::feedingFuel,
                (delta) -> hopper.changeFuelCountBy(delta));
        // climber = new Climber(new ClimberIOReal());
        climber = new Climber(new ClimberIO() {});

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
                drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(
                //     "Camera0",
                //     VisionConstants.robotToCamera0,
                //     drive::getPose), // TODO: set these Transforms
                // new VisionIOPhotonVisionSim(
                //     "Camera1", VisionConstants.robotToCamera1, drive::getPose));
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new IntakeIOSim());
        hopper = new Hopper(new HopperIOSim());
        indexer = new Indexer(new IndexerIOSim());
        feeder = new Feeder(new FeederIOSim());
        turret =
            new Turret(
                new TurretIOSim(),
                drive::getPose,
                drive::getChassisSpeeds,
                feeder::feedingFuel,
                (delta) -> hopper.changeFuelCountBy(delta));
        climber = new Climber(new ClimberIOSim());

        // Register a robot for collision with fuel
        FuelSim.getInstance()
            .registerRobot(
                Distance.ofRelativeUnits(29.0, Inches).in(Meters), // from left to right
                Distance.ofRelativeUnits(26.0, Inches).in(Meters), // from front to back
                Distance.ofRelativeUnits(6.0, Inches).in(Meters), // from floor to top of bumpers
                drive::getPose, // Supplier<Pose2d> of robot pose
                drive::getChassisSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds

        FuelSim.getInstance()
            .registerIntake(
                Distance.ofRelativeUnits(13.0, Inches).in(Meters),
                Distance.ofRelativeUnits(16.0, Inches).in(Meters),
                Distance.ofRelativeUnits(-8.5, Inches).in(Meters),
                Distance.ofRelativeUnits(8.5, Inches)
                    .in(Meters), // robot-centric coordinates for bounding box
                () ->
                    intake
                        .isRunning(), // (optional) BooleanSupplier for whether the intake should be
                // active at a given moment
                () ->
                    hopper.changeFuelCountBy(
                        1)); // (optional) Runnable called whenever a fuel is in-took
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        indexer = new Indexer(new IndexerIO() {});
        feeder = new Feeder(new FeederIO() {});
        turret =
            new Turret(
                new TurretIO() {},
                drive::getPose,
                drive::getChassisSpeeds,
                feeder::feedingFuel,
                (delta) -> hopper.changeFuelCountBy(delta));
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    autoCommandBuilder =
        new AutoCommandBuilder(drive, vision, intake, hopper, indexer, feeder, turret, climber);
    useDynamicAuto = new LoggedDashboardChooser<>("Use DynamicAuto");
    useDynamicAuto.addDefaultOption("Yes, use DynamicAuto", true);
    useDynamicAuto.addOption("No, use Hard Autos", false);
    hardAutoChooser = new LoggedDashboardChooser<>("Hard Autos", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> -controller.getRightY()));

    turret.setDefaultCommand(turret.fullFieldAim());
    // turret.setDefaultCommand(turret.calibrate());

    // Configure the button bindings
    configureButtonBindings();

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    CommandScheduler.getInstance()
        .schedule(
            StateLoggingCommands.logMechanisms(intake, hopper, indexer, feeder, turret, climber));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // #################### DRIVER CONTROLLER ####################

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.leftTrigger(0.1).onTrue(intake.start());

    controller.leftTrigger(0.1).onFalse(intake.stop());

    controller
        .rightTrigger(0.95)
        .onTrue(
            Commands.sequence(
                vision.setWhiteList(FieldConstants.kHubTags),
                Commands.repeatingSequence(
                    Commands.waitUntil(() -> turret.isReadyForFuel()),
                    feeder.start(),
                    Commands.waitUntil(() -> !turret.isReadyForFuel()),
                    feeder.stop())));
    controller.rightTrigger(0.9).onFalse(Commands.parallel(feeder.stop(), vision.clearWhiteList()));

    controller.leftBumper().onTrue(climber.retract());

    // #################### BUTTON BOX ####################

    Command leftClimbCommand = null;
    Command rightClimbCommand = null;
    Command feedOutpostCommand = null;
    try {
      leftClimbCommand =
          AutoDriveCommands.driveToPoseThenPath(
              drive, PathPlannerPath.fromPathFile("Left Climb"), true);
      rightClimbCommand =
          AutoDriveCommands.driveToPoseThenPath(
              drive, PathPlannerPath.fromPathFile("Right Climb"), true);
      feedOutpostCommand =
          AutoDriveCommands.driveToPoseThenPath(
              drive, PathPlannerPath.fromPathFile("Feed Outpost"), false);
    } catch (FileVersionException | IOException | ParseException e) {
      Elastic.sendNotification(
          new Notification(
              Elastic.NotificationLevel.WARNING,
              "Setting AutoDrive Button Bidings Crash",
              "Couldn't find the file for one of paths! Falling back to simple moveToPose."));
      leftClimbCommand =
          leftClimbCommand == null
              ? AutoDriveCommands.driveToPose(drive, FieldConstants.kLeftClimb, true)
              : leftClimbCommand;
      rightClimbCommand =
          rightClimbCommand == null
              ? AutoDriveCommands.driveToPose(drive, FieldConstants.kRightClimb, true)
              : rightClimbCommand;
      feedOutpostCommand = feedOutpostCommand == null ? Commands.none() : feedOutpostCommand;
    }

    climbLeftSide.whileTrue(leftClimbCommand.alongWith(climber.extend()));
    climbRightSide.whileTrue(rightClimbCommand.alongWith(climber.extend()));

    feedOutpost.whileTrue(feedOutpostCommand);

    moveToLeftTrench.whileTrue(
        AutoDriveCommands.driveToPose(drive, FieldConstants.kLeftBumpLaunch, false));
    moveToRightTrench.whileTrue(
        AutoDriveCommands.driveToPose(drive, FieldConstants.kRightBumpLaunch, false));

    reverseFeeder.onTrue(feeder.reverse());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (useDynamicAuto.get()) {
      try {
        return autoCommandBuilder.buildAutoCommand();
      } catch (FileVersionException | IOException | ParseException e) {
        Elastic.sendNotification(
            new Notification(
                Elastic.NotificationLevel.WARNING,
                "DynamicAuto Crash",
                "Couldn't find the file for one of paths!"));
        return Commands.parallel(
            intake.start(),
            hopper.start(),
            indexer.start(),
            feeder.start(),
            turret.fullFieldAim(),
            climber.calibrate());
      }
    } else {
      return hardAutoChooser.get();
    }
  }

  public void enabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(3));
  }

  public void disabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(1));
  }
}
