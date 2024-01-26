package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.JoystickUtils;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static ProfiledPIDController orbitPID;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity = getDriveTranslation(xSupplier, ySupplier);

          // the rotational rate of the robot
          double omega = JoystickUtils.curveInput(omegaSupplier.getAsDouble(), DEADBAND);

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  public static Command orbitDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Translation2d target) {
    return Commands.run(
            () -> {
              Translation2d linearVelocity = getDriveTranslation(xSupplier, ySupplier);

              Rotation2d driveAngle = drive.getRotation();

              Rotation2d rotationTarget =
                  drive
                      .getPose()
                      .relativeTo(new Pose2d(target, new Rotation2d()))
                      .getTranslation()
                      .getAngle()
                      .rotateBy(Rotation2d.fromDegrees(180));

              double omega = -orbitPID.calculate(rotationTarget.minus(driveAngle).getRotations());

              // omega = JoystickUtils.curveInput(omega, 0);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      drive.getRotation()));

              Logger.recordOutput("Drive/Orbit/Robot Rotation", driveAngle);
              Logger.recordOutput(
                  "Drive/Orbit/Target Rotation",
                  new Pose2d(drive.getPose().getTranslation(), rotationTarget));
              Logger.recordOutput("Drive/Orbit/Target", target);
            },
            drive)
        .beforeStarting(
            () -> {
              // config the PID controller before starting the command
              orbitPID =
                  new ProfiledPIDController(
                      11.0, 0.2, 0.1, new Constraints(1.5, 0.75), 1.0 / 250.0);
              orbitPID.enableContinuousInput(-0.5, 0.5);
              orbitPID.setTolerance(10);
            });
  }

  public static Command zeroDrive(DriveSubsystem drive) {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  public static Translation2d getDriveTranslation(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Apply deadband, and curve joystick inputs
    double linearMagnitude =
        JoystickUtils.curveInput(
            // uses the hypotenuse of the joystick to control the velocity (this is the
            // distance from )
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);

    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calcaulate new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
