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

          // the rotational rate of the robot (half cuz rotating too fast sucks)
          double omega = JoystickUtils.curveInput(omegaSupplier.getAsDouble(), DEADBAND) * 0.5;

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

  /**
   * Uses a PID loop to make the robot face a point on the feild, while maintaing joydtick control
   * of x and y.
   *
   * @param drive the DriveSubsystem
   * @param xSupplier a joystick supplier for the x velocity of the robot
   * @param ySupplier a joystick supplier for the y velocity of the robot
   * @param target a Translation2d representing the point on the field the robot should target
   * @return a Command with the specified behavior
   */
  public static Command orbitDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Translation2d target) {
    return Commands.run(
            () -> {
              // gets a x and y control inputs from the joystick
              Translation2d linearVelocity = getDriveTranslation(xSupplier, ySupplier);

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
              orbitPID.setGoal(rotationTarget.getRotations());
              double omega = orbitPID.calculate(drive.getPose().getRotation().getRotations());

              Logger.recordOutput("Drive/Orbit/error", orbitPID.getPositionError());

              // send speeds to drive function
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      drive.getRotation()));

              Logger.recordOutput("Drive/Orbit/Robot Rotation", drive.getRotation());
              Logger.recordOutput(
                  "Drive/Orbit/Target Rotation",
                  new Pose2d(drive.getPose().getTranslation(), rotationTarget));
              Logger.recordOutput("Drive/Orbit/Target", target);
            },
            drive)
        .beforeStarting(
            () -> {
              // config the PID controller before starting the command
              orbitPID = new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(1.5, 0.75));
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

  public static Command zeroOdometry(DriveSubsystem drive) {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(new Translation2d(), drive.getRotation())), drive)
        .ignoringDisable(true);
  }

  public static Rotation2d getAngleDelta(Pose2d robotPose, Translation2d target) {
    Rotation2d rotationTarget =
        robotPose
            .relativeTo(new Pose2d(target, new Rotation2d()))
            .getTranslation()
            .getAngle()
            .rotateBy(Rotation2d.fromDegrees(180));
    return rotationTarget.minus(robotPose.getRotation());
  }

  /**
   * Creates a new Translation2d for driving based off of an x and y output percent. This
   * translation has been curved to make the stick feel smoother and offer more control at the low
   * end.
   *
   * @param xSupplier the joystick x input
   * @param ySupplier the joystick y input
   * @return a new translation ready to be used for driving
   */
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
