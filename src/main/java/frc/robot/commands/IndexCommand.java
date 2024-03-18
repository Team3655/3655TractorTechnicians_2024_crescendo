// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IndexCommand extends Command {

  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;

  /** Creates a new IndexCommand. */
  public IndexCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {

    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setState(IntakeState.INDEX);
    shooter.requestState(ShooterTargets.INDEX);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeState.IDLE);
    shooter.requestState(ShooterTargets.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getIndexDistanceMM() <= 330;
  }
}
