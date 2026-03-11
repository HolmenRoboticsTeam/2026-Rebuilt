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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
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
  private Indexer indexer;
  private Feeder feeder;
  private Turret turret;

  // Auto
  private final LoggedDashboardChooser<Boolean> autoSelectorType;
  private final LoggedDashboardChooser<Command> hardAutoChooser;
  private final Field2d autoField = new Field2d();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController buttonBox = new XboxController(1);

  // Switch 1
  private final JoystickButton firstAutoToggleUp = new JoystickButton(buttonBox, 1);
  private final JoystickButton firstAutoToggleDown = new JoystickButton(buttonBox, 2);

  // Switch 2
  private final JoystickButton secondAutoToggleUp = new JoystickButton(buttonBox, 3);
  private final JoystickButton secondAutoToggleDown = new JoystickButton(buttonBox, 4);

  // Switch 3
  private final JoystickButton thirdAutoToggleUp = new JoystickButton(buttonBox, 5);
  private final JoystickButton thirdAutoToggleDown = new JoystickButton(buttonBox, 6);

  // Switch 4 (Alliance Win Override)
  private final JoystickButton blueAutoWinnerOverrideToggleUp = new JoystickButton(buttonBox, 7);
  private final JoystickButton redAutoWinnerOverrideToggleDown = new JoystickButton(buttonBox, 8);

  // Top Row
  private final JoystickButton startIntake = new JoystickButton(buttonBox, 11);
  private final JoystickButton startIndexer = new JoystickButton(buttonBox, 12);
  private final JoystickButton startFeeder = new JoystickButton(buttonBox, 13);

  // Mid Row
  private final JoystickButton midRow1 = new JoystickButton(buttonBox, 14);
  private final JoystickButton midRow2 = new JoystickButton(buttonBox, 15);
  private final JoystickButton midRow3 = new JoystickButton(buttonBox, 16);

  // Bottom Row
  private final JoystickButton lowRow1 = new JoystickButton(buttonBox, 17);
  private final JoystickButton lowRow2 = new JoystickButton(buttonBox, 18);
  private final JoystickButton lowRow3 = new JoystickButton(buttonBox, 19);

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
                new VisionIOLimelight("limelight", () -> drive.getRotation()));
        intake = new Intake(new IntakeIOReal());
        indexer = new Indexer(new IndexerIOReal(), () -> true);
        feeder = new Feeder(new FeederIOReal(), () -> turret.isReadyForFuel());
        // intake = new Intake(new IntakeIO() {});
        // indexer = new Indexer(new IndexerIO() {}, () -> true);
        // feeder = new Feeder(new FeederIO() {}, () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIOReal(),
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> indexer.changeHeldFuelBy(d));
        // turret =
        //     new Turret(
        //         new TurretIO() {},
        //         () -> drive.getPose(),
        //         () -> drive.getChassisSpeeds(),
        //         () -> feeder.feedingFuel(),
        //         (d) -> indexer.changeHeldFuelBy(d));
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
        indexer = new Indexer(new IndexerIOSim(), () -> true);
        feeder = new Feeder(new FeederIOSim(), () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIOSim(),
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> indexer.changeHeldFuelBy(d));

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
                Distance.ofRelativeUnits(-8.5, Inches).in(Meters),
                Distance.ofRelativeUnits(8.5, Inches)
                    .in(Meters), // robot-centric coordinates for bounding box
                () -> {
                  return intake.isRunning()
                      && indexer.getHeldFuel() < IndexerConstants.Sim.maxHopperCapacity;
                }, // (optional) BooleanSupplier for whether the intake should be active at a given
                // moment
                () -> {
                  indexer.changeHeldFuelBy(1);
                }); // (optional) Runnable called whenever a fuel is in-took
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
        indexer = new Indexer(new IndexerIO() {}, () -> true);
        feeder = new Feeder(new FeederIO() {}, () -> turret.isReadyForFuel());
        turret =
            new Turret(
                new TurretIO() {},
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> feeder.feedingFuel(),
                (d) -> indexer.changeHeldFuelBy(d));
        break;
    }

    // Auto setup
    NamedCommands.registerCommands(
        Constants.getNamedCommand(drive, vision, intake, indexer, feeder, turret));
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> -controller.getRightY()));

    turret.setDefaultCommand(turret.fullFieldAim());
    // turret.setDefaultCommand(turret.calibrate());

    // intake.setDefaultCommand(intake.stop());
    // indexer.setDefaultCommand(indexer.stop());
    // feeder.setDefaultCommand(feeder.stop());

    // Auto Field
    SmartDashboard.putData("AutoField", autoField);
    CommandScheduler.getInstance()
        .schedule(
            AutoCommands.displayAutoField(autoField, () -> getAutoName(), () -> drive.getPose()));

    // Configure the button bindings
    configureButtonBindings();

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    CommandScheduler.getInstance()
        .schedule(StateLoggingCommands.logMechanisms(intake, indexer, feeder, turret));
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

    // Reset gyro to ZERO when Start button is pressed
    controller.start().onTrue(Commands.runOnce(() -> drive.resetGyro(Rotation2d.kZero)));

    // Resets gyro to the next vision MT1 measurement.
    // boolean[] hasResetGyro = new boolean[1];
    // controller
    //     .start()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   vision.setRotationConsumer(
    //                       (r) -> {
    //                         drive.resetGyro(r);
    //                         hasResetGyro[0] = true;
    //                       });
    //                 }),
    //             Commands.waitUntil(() -> hasResetGyro[0]),
    //             Commands.runOnce(
    //                 () -> {
    //                   vision.setRotationConsumer((r) -> {});
    //                   hasResetGyro[0] = false;
    //                 })));

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

    // #################### BUTTON BOX ####################

    controller.leftTrigger(0.9).onTrue(intake.start());
    controller.leftTrigger(0.9).onFalse(intake.stop());

    controller.a().onTrue(indexer.start());
    controller.a().onFalse(indexer.stop());

    controller.b().onTrue(feeder.reverse());
    controller.b().onFalse(feeder.stop());

    controller.y().onTrue(feeder.start());
    controller.y().onFalse(feeder.stop());

    // Shift Overriding
    blueAutoWinnerOverrideToggleUp.onTrue(
        Commands.runOnce(
            () -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(Alliance.Blue))));
    blueAutoWinnerOverrideToggleUp.onFalse(
        Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty())));
    redAutoWinnerOverrideToggleDown.onTrue(
        Commands.runOnce(
            () -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(Alliance.Red))));
    redAutoWinnerOverrideToggleDown.onFalse(
        Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.empty())));
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
          firstAutoToggleUp.getAsBoolean()
              ? "Left"
              : (firstAutoToggleDown.getAsBoolean() ? "Right" : "Hub"),
          secondAutoToggleUp.getAsBoolean()
              ? "Up"
              : (secondAutoToggleDown.getAsBoolean() ? "Down" : "Mid"),
          thirdAutoToggleUp.getAsBoolean()
              ? "Up"
              : (thirdAutoToggleDown.getAsBoolean() ? "Down" : "Mid")
        };

    return autonomousData[0] + "-" + autonomousData[1] + "-" + autonomousData[2];
  }

  public void enabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(4));
  }

  public void disabledInit() {
    CommandScheduler.getInstance().schedule(vision.setIMUMode(1));
  }
}
