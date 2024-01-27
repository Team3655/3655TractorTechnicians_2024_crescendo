// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class OrbitAndShootCommand extends Command {

  private ProfiledPIDController pid;

  /** Creates a new OrbitAndShootCommand. */
  public OrbitAndShootCommand(
    DoubleSupplier xSupplier, 
    DoubleSupplier ySupplier, 
    Translation2d target,
    DriveSubsystem drive, 
    ShooterSubsystem shooter) {
    addRequirements(drive, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new ProfiledPIDController(
    10, 
    0, 
    0, 
    new Constraints(
      Units.degreesToRotations(90), 
      Units.degreesToRotations(60)), 
    0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
