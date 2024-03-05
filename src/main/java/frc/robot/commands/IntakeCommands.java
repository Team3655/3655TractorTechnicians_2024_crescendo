// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Add your docs here. */
public class IntakeCommands {

  public static Command deploy(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> {
              intake.setStageOne(Value.kForward);
              intake.setVoltage(10.0);
            },
            intake)
        .andThen(new WaitCommand(0.1))
        .andThen(Commands.runOnce(() -> intake.setStageTwo(Value.kForward), intake));
  }

  public static Command retract(IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> {
              intake.setStageTwo(Value.kReverse);
              intake.setVoltage(0.0);
            },
            intake)
        .andThen(new WaitCommand(0.1))
        .andThen(Commands.runOnce(() -> intake.setStageOne(Value.kReverse), intake));
  }
}
