package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindingCommands;
import frc.robot.commands.ShooterOrbit;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXPro;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CommandNXT;
import frc.robot.util.config.CharacterizationConfiguration;
import frc.robot.util.config.PortConfiguration;
import frc.robot.util.config.RobotConfigurations;
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

  // Controller
  private final CommandNXT driveJoystick = new CommandNXT(0);
  private final CommandJoystick turnJoystick = new CommandJoystick(1);
  private final CommandXboxController controller = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 5000);

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
                    characterizationConfig.frontLeftOffset.getRotations()),
                new ModuleIOTalonFXPro(
                    portConfig.frontRightDriveMotorID,
                    portConfig.frontRightTurnMotorID,
                    portConfig.frontRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.frontRightOffset.getRotations()),
                new ModuleIOTalonFXPro(
                    portConfig.backLeftDriveMotorID,
                    portConfig.backLeftTurnMotorID,
                    portConfig.backLeftAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.backLeftOffset.getRotations()),
                new ModuleIOTalonFXPro(
                    portConfig.backRightDriveMotorID,
                    portConfig.backRightTurnMotorID,
                    portConfig.backRightAbsoluteEncoderID,
                    portConfig.driveCANBus,
                    characterizationConfig.backRightOffset.getRotations()),
                vision);

        shooter =
            new ShooterSubsystem(
                new ShooterIOSpark(
                    portConfig.topFlywheelMoterID,
                    portConfig.bottomFlywheelMotorID,
                    portConfig.kickerMotorID,
                    portConfig.pivotMotorID));

        intake =
            new IntakeSubsystem(
                new IntakeIOHardware(
                    portConfig.pneumaticHubID,
                    portConfig.intakeMotorID,
                    portConfig.deploySolenoidPort,
                    portConfig.intakeBeamBreakPort));
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
        break;
    }

    // Set up named commands for PathPlanner

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
            () -> turnJoystick.getX() + controller.getRightX()));
    // () -> -controller.getRawAxis(2))); // MacOS

    controller
        .rightBumper()
        .or(driveJoystick.fireStage1())
        .whileTrue(
            new ShooterOrbit(
                drive,
                shooter,
                () -> -driveJoystick.getY() - controller.getLeftY(),
                () -> -driveJoystick.getX() - controller.getLeftX(),
                () -> driveJoystick.fireStage2().getAsBoolean(),
                new Translation2d(0.0, 5.5))); // 0.0, 5.5

    // driveJoystick.d1().whileTrue(Commands.run(drive::stopWithX, drive));

    driveJoystick.b1().or(controller.back()).onTrue(DriveCommands.zeroDrive(drive));

    controller
        .start()
        .or(turnJoystick.button(2))
        .onTrue(
            DriveCommands.zeroOdometry(
                drive, new Translation2d(Units.inchesToMeters(36 + (32.0 / 2.0)), 5.55)));

    controller
        .leftBumper()
        .whileTrue(
            PathfindingCommands.pathfindToPose(
                new Pose2d(1.88, 7.70, Rotation2d.fromDegrees(90)), 0, 0));
    // controller.button(1).onTrue(DriveCommands.zeroDrive(drive)); // MacOS

    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setAngle(Rotation2d.fromDegrees(60));
                },
                shooter));

    controller
        .povUp()
        .or(turnJoystick.button(5))
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.jogAngle(-1.0);
                },
                shooter));

    controller
        .povDown()
        .or(turnJoystick.button(3))
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.jogAngle(1.0);
                },
                shooter));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setAngle(Rotation2d.fromDegrees(0));
                },
                shooter));

    controller.y().whileTrue(ShootingCommands.holdShoot(shooter, flywheelSpeedInput::get));

    driveJoystick
        .firePaddleUp()
        .or(controller.povLeft())
        .whileTrue(
            Commands.startEnd(
                () -> intake.setState(IntakeSubsystem.INTAKE_STATE),
                () -> intake.setState(IntakeSubsystem.TUCKED_STATE),
                intake));

    controller
        .x()
        .or(driveJoystick.fireStage2())
        .or(driveJoystick.firePaddleDown())
        .whileTrue(
            Commands.startEnd(() -> intake.setVoltage(10), () -> intake.setVoltage(0), intake));

    driveJoystick.a2().whileTrue(ShootingCommands.holdShoot(shooter, flywheelSpeedInput::get));

    // shooter intake
    // driveJoystick
    // .fireStage1()
    // .whileTrue(
    // Commands.startEnd(
    // () -> {
    // shooter.runVelocity(-1000);
    // shooter.setKicker(-12.0);
    // },
    // () -> {
    // shooter.stopFlywheel();
    // shooter.setKicker(0.0);
    // },
    // shooter));

    // driveJoystick
    // .button(CommandNXT.A2)
    // .whileTrue(
    // Commands.startEnd(
    // () -> shooter.runVelocity(flywheelSpeedInput.get()),
    // shooter::stopFlywheel,
    // shooter));
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
