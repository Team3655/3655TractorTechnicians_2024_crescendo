// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Add your docs here. */
public class IntakeCommands {

  public static Command ShooterIntakeSequence(ShooterSubsystem shooter, IntakeSubsystem intake) {
    return new ShooterIntake(shooter, intake)
        .andThen(new WaitCommand(0.33))
        .andThen(
            Commands.runOnce(
                () -> {
                  intake.setState(IntakeState.IDLE);
                  shooter.requestState(ShooterTargets.IDLE);
                },
                intake));
  }

  public static Command deploy(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.STAGE_ONE);
            },
            intake)
        .andThen(new WaitCommand(0.2))
        .andThen(Commands.runOnce(() -> intake.setState(IntakeState.FULL_DEPLOY), intake));
  }

  public static Command retract(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.STAGE_ONE);
            },
            intake)
        .andThen(new WaitCommand(0.5))
        .andThen(Commands.runOnce(() -> intake.setState(IntakeState.IDLE), intake));
  }

  public static Command intakeSearch(IntakeSubsystem intake) {
    return deploy(intake)
        .andThen(new WaitUntilCommand(() -> intake.getProximity()))
        .andThen(retract(intake));
  }
}
