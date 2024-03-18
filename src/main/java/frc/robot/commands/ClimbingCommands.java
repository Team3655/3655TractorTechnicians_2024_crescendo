// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class ClimbingCommands {

  // public static Command prepClimb(ClimberSubsystem climber, ShooterSubsystem shooter) {
  //   return Commands.runOnce(
  //       () -> {
  //         climber.setAngle(Rotation2d.fromDegrees(110.0));
  //         shooter.setAngle(Rotation2d.fromDegrees(5.0));
  //       },
  //       climber,
  //       shooter);
  // }

  // public static Command climb(ClimberSubsystem climber, ShooterSubsystem shooter) {
  //   return Commands.runOnce(
  //           () -> {
  //             climber.setAngle(Rotation2d.fromDegrees(0.0));
  //             shooter.setAngle(Rotation2d.fromDegrees(30.0));
  //           },
  //           climber,
  //           shooter)
  //       .andThen(
  //           new WaitCommand(1.0)
  //               .andThen(
  //                   () -> {
  //                     shooter.setAngle(Rotation2d.fromDegrees(18.0));
  //                   },
  //                   climber));
  // }
}
