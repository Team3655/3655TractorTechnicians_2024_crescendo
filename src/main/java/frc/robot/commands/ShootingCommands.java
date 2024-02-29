// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Add your docs here. */
public class ShootingCommands {

  private ShootingCommands() {}

  public Command ShootWhenReady(ShooterSubsystem shooter) {
    return Commands.run(() -> {}, shooter);
  }

  public static Command adjustShooter(ShooterSubsystem shooter, Supplier<Pose2d> pose) {
    return Commands.run(
        () -> {
          Translation2d reletiveTarget =
              pose.get()
                  // get the drive position reletive to the target position
                  .relativeTo(new Pose2d(ShooterOrbit.target, new Rotation2d()))
                  // get as a vector
                  .getTranslation();
          shooter.setShooterAngleFromDist(reletiveTarget.getNorm());
        },
        shooter);
  }

  public static Command stopShooter(ShooterSubsystem shooter) {
    return Commands.runOnce(() -> shooter.stopFlywheel(), shooter);
  }

  public static Command runKicker(ShooterSubsystem shooter) {
    return Commands.startEnd(
        () -> {
          shooter.setKicker(12.0);
        },
        () -> {
          shooter.setKicker(0.0);
        },
        shooter);
  }

  public static Command setSpeed(ShooterSubsystem shooter, double rpm) {
    return Commands.runOnce(() -> shooter.runVelocity(rpm), shooter);
  }

  public static Command holdShoot(ShooterSubsystem shooter, DoubleSupplier rpm) {
    return Commands.startEnd(
        () -> {
          shooter.runVelocity(rpm.getAsDouble());
          shooter.setKicker(12.0);
        },
        () -> {
          shooter.stopFlywheel();
          shooter.setKicker(0.0);
        },
        shooter);
  }
}
