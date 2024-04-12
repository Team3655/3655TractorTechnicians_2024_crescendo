// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class TrapCommand extends Command {
  private ShooterSubsystem shooter;
  private ClimberSubsystem climber;
  private IntakeSubsystem intake;
  private BooleanSupplier kickerRequest;
  /** Creates a new TrapCommand. */
  public TrapCommand(
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      IntakeSubsystem intake,
      BooleanSupplier kickerRequest) {
    this.shooter = shooter;
    this.climber = climber;
    this.intake = intake;
    this.kickerRequest = kickerRequest;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, climber, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.requestState(ShooterTargets.TRAP);
    climber.setAngle(Rotation2d.fromDegrees(0.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kickerRequest.getAsBoolean()) {
      intake.setState(IntakeState.FORWARD_FEED);
    } else {
      intake.setState(IntakeState.IDLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeState.IDLE);
    shooter.requestState(ShooterTargets.IDLE);
    climber.setAngle(Rotation2d.fromDegrees(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
