// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class Flywheels {

  private final FlywheelIO rightIO;
  private final FlywheelIO leftIO;

  public Flywheels(FlywheelIO leftIO, FlywheelIO rightIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;

  }

  public void setVelocity(double leftVel, double rightVel) {
    leftIO.setVelocity(leftVel, 0);
    rightIO.setVelocity(rightVel, 0);
  }

}
