// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTarget;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
          Translation2d relativeTarget =
              pose.get()
                  // get the drive position reletive to the target position
                  .relativeTo(new Pose2d(ShooterOrbit.blueTarget, new Rotation2d()))
                  // get as a vector
                  .getTranslation();
          shooter.setShooterAngleFromDist(relativeTarget.getNorm());
        },
        shooter);
  }

  public static Command runKicker(IntakeSubsystem intake) {
    return Commands.startEnd(
        () -> {
          intake.setState(IntakeState.FORWARD_FEED);
        },
        () -> {
          intake.setState(IntakeState.IDLE);
        },
        intake);
  }

  public static Command requestState(ShooterSubsystem shooter, ShooterTarget state) {
    return Commands.runOnce(() -> shooter.requestState(state), shooter);
  }

  public static Command ampShoot(
      ShooterSubsystem shooter, ClimberSubsystem climber, IntakeSubsystem intake) {
    return Commands.runOnce(
            () -> {
              shooter.requestState(ShooterTargets.AMP);
              climber.setAngle(Rotation2d.fromDegrees(105.0));
            },
            shooter,
            climber)
        .andThen(new WaitCommand(1.00))
        .andThen(
            Commands.run(
                () -> {
                  intake.setState(IntakeState.FORWARD_FEED);
                },
                shooter));
  }

  // public static Command jogZero(ShooterSubsystem shooter, Rotation2d angle) {
  //   return Commands.runOnce(() -> shooter.jogZero(angle), shooter);
  // }
}
