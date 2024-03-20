// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class ClimbingCommands {

  public static Command prepClimb(ClimberSubsystem climber, ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          climber.setAngle(Rotation2d.fromDegrees(125.0));
          shooter.requestState(ShooterTargets.PREP_CLIMB);
        },
        climber,
        shooter);
  }

  public static Command climb(ClimberSubsystem climber, ShooterSubsystem shooter) {
    return Commands.runOnce(
            () -> {
              climber.setAngle(Rotation2d.fromDegrees(0.0));
              shooter.requestState(ShooterTargets.CLIMB_STAGE_ONE);
            },
            climber,
            shooter)
        .andThen(
            new WaitCommand(1.0)
                .andThen(
                    () -> {
                      shooter.requestState(ShooterTargets.CLIMBE_STAGE_TWO);
                    },
                    climber));
  }
}
