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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterOrbit extends Command {

  private DriveSubsystem drive;
  private ShooterSubsystem shooter;

  private ProfiledPIDController turnFeedback;

  private Translation2d target;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  /** Creates a new OrbitAndShootCommand. */
  public ShooterOrbit(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Translation2d target) {

    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.target = target;

    turnFeedback = new ProfiledPIDController(3.0, 0.0012, 0.0, new Constraints(0.0, 0.0));
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

    Translation2d reletiveTarget =
        drive
            .getPose()
            // get the drive position reletive to the target position
            .relativeTo(new Pose2d(target, new Rotation2d()))
            // get as a vector
            .getTranslation();

    Rotation2d rotationTarget =
        reletiveTarget
            // get the angle of the vector
            .getAngle()
            // rotate the angle by 180 because we want the robot to face down the vector
            .rotateBy(Rotation2d.fromDegrees(180));

    // calculate pid output based on the delta to target rotation
    turnFeedback.setGoal(rotationTarget.getRotations());
    double omega = turnFeedback.calculate(drive.getPose().getRotation().getRotations());

    shooter.setShooterAngleFromDist(reletiveTarget.getNorm());

    // send speeds to drive function
    drive.setTargetVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxVelocityMetersPerSec(),
            linearVelocity.getY() * drive.getMaxVelocityMetersPerSec(),
            omega * drive.getMaxAngularVelocityRadPerSec(),
            drive.getPose().getRotation()));

    Logger.recordOutput("Drive/Orbit/Distance To Target", reletiveTarget.getNorm());
    Logger.recordOutput("Drive/Orbit/Error", turnFeedback.getPositionError());
    Logger.recordOutput("Drive/Orbit/Robot Rotation", drive.getPose().getRotation());
    Logger.recordOutput(
        "Drive/Orbit/Target Rotation",
        new Pose2d(drive.getPose().getTranslation(), rotationTarget));
    Logger.recordOutput("Drive/Orbit/Target", target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
