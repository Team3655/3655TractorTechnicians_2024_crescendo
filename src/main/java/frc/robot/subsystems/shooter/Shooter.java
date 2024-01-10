// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final Flywheels flywheels;

  /** Creates a new shooter. */
  public Shooter(FlywheelIO leftIo, FlywheelIO rightIo) {
    flywheels = new Flywheels(leftIo, rightIo);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheels.periodic();
  }

  public void runVelocity(double leftVel, double rightVel) {
    flywheels.setVelocity(leftVel, rightVel);
  }

  public void stop() {
    flywheels.stop();
  }
}
