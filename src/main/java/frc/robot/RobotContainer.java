package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BabyBirdCommand;
import frc.robot.commands.ClimbingCommands;
import frc.robot.commands.DeadReckoningCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterOrbit;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.TrapCommand;
import frc.robot.config.CharacterizationConfiguration;
import frc.robot.config.PortConfiguration;
import frc.robot.config.RobotConfigurations;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXPro;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIODouble;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;
import java.io.IOException;
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
  private final DriveSubsystem drive;
  private final VisionSubsystem vision;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final ClimberSubsystem climber;

  // Controller
  private final CommandNXT driveJoystick = new CommandNXT(0);
  private final CommandJoystick turnJoystick = new CommandJoystick(1);
  private final CommandGenericHID tractorController = new CommandGenericHID(2);
  private final CommandXboxController controller = new CommandXboxController(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @throws IOException
   * @throws IllegalAccessException
   */
  public RobotContainer() throws IOException {

    switch (Constants.currentMode) {
      case REAL:
        PortConfiguration portConfig =
            RobotConfigurations.getPortConfiguration(Constants.currentVersion);
        CharacterizationConfiguration characterizationConfig =
            RobotConfigurations.getCharacterizationConfiguration(Constants.currentVersion);

        // Real robot, instantiate hardware IO implementations
        vision = new VisionSubsystem(new VisionIOLimelight(portConfig.limelightName));

        drive =
            new DriveSubsystem(
                new GyroIOPigeon2(portConfig.gyroID, 0, 0.0, portConfig.driveCANBus),
                new ModuleIOTalonFXPro(
                    portConfig.frontLeftDriveMotorID,
                    portConfig.frontLeftTurnMotorID,
                    portConfig.frontLeftAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.frontLeftOffset,
                    characterizationConfig.driveGearRatio,
                    characterizationConfig.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.frontRightDriveMotorID,
                    portConfig.frontRightTurnMotorID,
                    portConfig.frontRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.frontRightOffset,
                    characterizationConfig.driveGearRatio,
                    characterizationConfig.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.backLeftDriveMotorID,
                    portConfig.backLeftTurnMotorID,
                    portConfig.backLeftAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.backLeftOffset,
                    characterizationConfig.driveGearRatio,
                    characterizationConfig.maxVelocity),
                new ModuleIOTalonFXPro(
                    portConfig.backRightDriveMotorID,
                    portConfig.backRightTurnMotorID,
                    portConfig.backRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.backRightOffset,
                    characterizationConfig.driveGearRatio,
                    characterizationConfig.maxVelocity),
                vision);

        shooter =
            new ShooterSubsystem(
                new ShooterIOSpark(
                    portConfig.topFlywheelMoterID,
                    portConfig.bottomFlywheelMotorID,
                    portConfig.feedMotorID,
                    portConfig.pivotMotorID));

        intake =
            new IntakeSubsystem(
                new IntakeIODouble(
                    portConfig.pneumaticHubID,
                    portConfig.intakeMotorID,
                    portConfig.feedMotorID,
                    portConfig.indexDistanceID,
                    portConfig.stageOneForwardPort,
                    portConfig.stageOneReversePort,
                    portConfig.stageTwoForwardPort,
                    portConfig.stageTwoReversePort,
                    portConfig.intakeBeamBreakPort));

        climber =
            new ClimberSubsystem(
                new ClimberIOSpark(portConfig.rightClimberMotorID, portConfig.leftClimberMotorID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new VisionSubsystem(new VisionIO() {});

        drive =
            new DriveSubsystem(
                null,
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                vision);

        shooter = new ShooterSubsystem(new ShooterIOSim());

        intake = new IntakeSubsystem(new IntakeIOSim());

        climber = new ClimberSubsystem(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new VisionSubsystem(new VisionIO() {});

        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);

        intake = new IntakeSubsystem(new IntakeIO() {});

        shooter = new ShooterSubsystem(new ShooterIO() {});

        climber = new ClimberSubsystem(new ClimberIO() {});
        break;
    }

    // Set up named commands for PathPlanner
    NamedCommands.registerCommand(
        "ShooterLeaveLine",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.LEAVE_LINE, () -> false));
    NamedCommands.registerCommand(
        "ShooterLeaveLineShoot",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.LEAVE_LINE, () -> true));
    NamedCommands.registerCommand(
        "ShooterSubwofer",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.SUBWOFER, () -> false));
    NamedCommands.registerCommand(
        "ShooterSubwoferShoot",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.SUBWOFER, () -> true));
    NamedCommands.registerCommand(
        "ShooterPodium",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.BEHIND_PODIUM, () -> false));
    NamedCommands.registerCommand(
        "ShooterPodiumShoot",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.BEHIND_PODIUM, () -> true));
    NamedCommands.registerCommand(
        "ShooterLong",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.AUTO_LONG, () -> false));
    NamedCommands.registerCommand(
        "ShooterLongShoot",
        new DeadReckoningCommand(shooter, intake, ShooterTargets.AUTO_LONG, () -> true));
    NamedCommands.registerCommand("RunKicker", ShootingCommands.runKicker(intake).withTimeout(1.0));
    NamedCommands.registerCommand("DeployIntake", IntakeCommands.intakeSearch(intake));
    NamedCommands.registerCommand("RetractIntake", IntakeCommands.retract(intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    // "Drive FF Characterization",
    // new FeedForwardCharacterization(
    // drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button -> command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            // multiply by (1 - RightTrigger) to act as a variable "brake" or "damper" on
            // the robots zoomieness
            () -> -driveJoystick.getY() - controller.getLeftY(),
            () -> -driveJoystick.getX() - controller.getLeftX(),
            () -> -turnJoystick.getX() - controller.getRightX()));

    controller
        .rightBumper()
        .or(driveJoystick.fireStage1())
        .whileTrue(
            new ShooterOrbit(
                drive,
                shooter,
                intake,
                () -> -driveJoystick.getY() - controller.getLeftY(),
                () -> -driveJoystick.getX() - controller.getLeftX(),
                () -> driveJoystick.fireStage2().getAsBoolean(),
                Optional.empty()));

    tractorController
        .button(6)
        .whileTrue(
            new ShooterOrbit(
                drive,
                shooter,
                intake,
                () -> -driveJoystick.getY() - controller.getLeftY(),
                () -> -driveJoystick.getX() - controller.getLeftX(),
                () -> turnJoystick.button(1).getAsBoolean(),
                Optional.of(Rotation2d.fromDegrees(54.0))));

    driveJoystick.b1().or(controller.back()).onTrue(DriveCommands.zeroDrive(drive));

    turnJoystick
        .button(2)
        .or(controller.start())
        .onTrue(
            DriveCommands.zeroOdometry(
                drive, new Translation2d(Units.inchesToMeters(36 + (32.0 / 2.0)), 5.55)));

    tractorController
        .button(5)
        .whileTrue(ShootingCommands.ampShoot(shooter, climber, intake))
        .onFalse(
            Commands.run(
                () -> {
                  shooter.requestState(ShooterTargets.IDLE);
                  climber.setAngle(new Rotation2d());
                  intake.setState(IntakeState.IDLE);
                },
                intake,
                shooter,
                climber));

    tractorController.button(18).whileTrue(new BabyBirdCommand(shooter, intake));

    controller
        .povUp()
        .or(turnJoystick.button(5))
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.jogAngleOffset(-0.5);
                },
                shooter));

    controller
        .povDown()
        .or(turnJoystick.button(3))
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.jogAngleOffset(0.5);
                },
                shooter));

    driveJoystick
        .firePaddleUp()
        .or(controller.povLeft())
        .onTrue(IntakeCommands.intakeSearch(intake))
        .onFalse(
            IntakeCommands.retract(intake)
                .andThen(new WaitCommand(0.25))
                .andThen(new IndexCommand(shooter, intake))
                .withTimeout(4));

    driveJoystick
        .a2()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intake.setState(IntakeState.REVERSE_FEED);
                },
                () -> {
                  intake.setState(IntakeState.IDLE);
                },
                intake));

    tractorController
        .button(11)
        .whileTrue(
            new DeadReckoningCommand(
                shooter, intake, ShooterTargets.SUBWOFER, () -> driveJoystick.d1().getAsBoolean()));

    tractorController
        .button(13)
        .whileTrue(
            new DeadReckoningCommand(
                shooter,
                intake,
                ShooterTargets.LEAVE_LINE,
                () -> driveJoystick.d1().getAsBoolean()));

    tractorController
        .button(15)
        .whileTrue(
            new DeadReckoningCommand(
                shooter,
                intake,
                ShooterTargets.BEHIND_PODIUM,
                () -> driveJoystick.d1().getAsBoolean()));

    tractorController
        .button(24)
        .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.OFF), intake));

    tractorController
        .button(9)
        .whileTrue(
            new TrapCommand(shooter, climber, intake, () -> driveJoystick.d1().getAsBoolean()));

    tractorController
        .button(1)
        .or(controller.y())
        .onTrue(ClimbingCommands.prepClimb(climber, shooter))
        .onFalse(ClimbingCommands.climb(climber, shooter));

    // tractorController
    //     .button(22)
    //     .onTrue(ShootingCommands.jogZero(shooter, Rotation2d.fromDegrees(-1.0)));

    // tractorController
    //     .button(23)
    //     .onTrue(ShootingCommands.jogZero(shooter, Rotation2d.fromDegrees(1.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
