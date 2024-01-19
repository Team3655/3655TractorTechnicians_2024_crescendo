package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.JoystickUtils;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband, and curve joystick inputs
          double linearMagnitude = JoystickUtils.curveInput(
              // uses the hypotenuse of the joystick to control the velocity (this is the
              // distance from )
              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);

          Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          //
          double omega = JoystickUtils.curveInput(omegaSupplier.getAsDouble(), DEADBAND);

          // Calcaulate new linear velocity
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

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

  public static Command zeroDrive(DriveSubsystem drive) {
    return Commands.runOnce(() -> drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())), drive).ignoringDisable(true);
  }
}
