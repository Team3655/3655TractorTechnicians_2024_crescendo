// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTarget;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class DeadReckoningCommand extends Command {

  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;
  private ShooterTarget state;
  private BooleanSupplier kickerRequest;

  /** Creates a new DeadReckoningCommand. */
  public DeadReckoningCommand(
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      ShooterTarget state,
      BooleanSupplier kickerRequest) {

    this.shooter = shooter;
    this.intake = intake;
    this.state = state;
    this.kickerRequest = kickerRequest;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.requestState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kickerRequest.getAsBoolean()) intake.setState(IntakeState.FORWARD_FEED);
    else intake.setState(IntakeState.IDLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.requestState(ShooterTargets.IDLE);
    intake.setState(IntakeState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
