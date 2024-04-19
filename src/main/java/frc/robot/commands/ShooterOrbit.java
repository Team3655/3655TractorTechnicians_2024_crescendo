// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTarget;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterOrbit extends Command {

  private static final double KP = 3.5;
  private static final double KI = 0.0012;

  private static final double LATANCY_SEC = 0.275;

  public static Translation2d blueTarget = new Translation2d(0.0, 5.55);
  public static Translation2d redTarget = new Translation2d(16.535, 5.55);

  private DriveSubsystem drive;
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;

  private ProfiledPIDController turnFeedback;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private BooleanSupplier kickSupplier;

  private Optional<Rotation2d> angle;

  /** Creates a new OrbitAndShootCommand. */
  public ShooterOrbit(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier kickRequest,
      Optional<Rotation2d> angle) {

    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.kickSupplier = kickRequest;
    this.angle = angle;

    turnFeedback = new ProfiledPIDController(KP, KI, 0.0, new Constraints(0.0, 0.0));
    turnFeedback.enableContinuousInput(-0.5, 0.5);

    addRequirements(drive, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets a x and y control inputs from the joystick
    Translation2d linearVelocity = DriveCommands.getDriveTranslation(xSupplier, ySupplier);

    Pose2d projectedPose = drive.getProjectedPose(LATANCY_SEC, true);

    Translation2d reletiveTarget =
        projectedPose
            // get the drive position reletive to the target position
            .relativeTo(new Pose2d(getMirroredTarget(), new Rotation2d()))
            // get as a vector
            .getTranslation();

    // adjust offset to ensure robot shoots into target center
    Rotation2d rotationTarget =
        reletiveTarget
            .getAngle()
            // .plus(Rotation2d.fromDegrees(180))
            .plus(Rotation2d.fromDegrees(3.5));

    // calculate pid output based on the delta to target rotation
    turnFeedback.setGoal(rotationTarget.getRotations());
    double omega = turnFeedback.calculate(drive.getPose(true).getRotation().getRotations());

    // send speeds to drive function
    drive.setTargetVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxVelocityMetersPerSec(),
            linearVelocity.getY() * drive.getMaxVelocityMetersPerSec(),
            omega * drive.getMaxAngularVelocityRadPerSec(),
            drive.getPose().getRotation()));

    if (angle.isEmpty()) {
      shooter.setShooterAngleFromDist(reletiveTarget.getNorm());
    } else {
      shooter.requestState(new ShooterTarget(angle.get(), Optional.of(ShooterTargets.SPEAKER_RPM)));
    }

    if (kickSupplier.getAsBoolean()) {
      intake.setState(IntakeState.FORWARD_FEED);
    } else {
      intake.setState(IntakeState.IDLE);
    }

    Logger.recordOutput("Drive/Orbit/Distance To Target", reletiveTarget.getNorm());
    Logger.recordOutput("Drive/Orbit/Error", turnFeedback.getPositionError());
    Logger.recordOutput("Drive/Orbit/Projected Pose", projectedPose);
    Logger.recordOutput(
        "Drive/Orbit/Target Rotation",
        new Pose2d(drive.getPose().getTranslation(), rotationTarget));
    Logger.recordOutput("Drive/Orbit/Target", getMirroredTarget());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.requestState(ShooterTargets.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  public Translation2d getMirroredTarget() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }
}
