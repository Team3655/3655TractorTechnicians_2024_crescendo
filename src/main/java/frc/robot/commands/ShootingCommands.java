// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShootingCommands {

  private ShootingCommands() {}

  public Command ShootWhenReady(ShooterSubsystem shooter) {
    return Commands.run(() -> {}, shooter);
  }

  public Command Shoot(ShooterSubsystem shooter) {
    return Commands.run(() -> {}, shooter);
  }

  public static Command holdShoot(ShooterSubsystem shooter, DoubleSupplier speed) {
    return Commands.startEnd(
        () -> {
          shooter.runVelocity(speed.getAsDouble());
          shooter.setKicker(12.0);
        },
        () -> {
          shooter.stopFlywheel();
          shooter.setKicker(0.0);
        },
        shooter);
  }
}
