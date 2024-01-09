// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.Constants;

/** Add your docs here. */
public class Flywheels {

  private final FlywheelIO leftIO;
  private final FlywheelIO rightIO;
  private final FlywheelIOInputsAutoLogged leftInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged rightInputs = new FlywheelIOInputsAutoLogged();

  public Flywheels(FlywheelIO leftIO, FlywheelIO rightIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;

    double p;
    switch (Constants.currentMode) {
      case REAL:
        p = 1;
        rightIO.configurePID(p, 0, 0);
        leftIO.configurePID(p, 0, 0);
        break;
      case SIM:
        p = 1;
        rightIO.configurePID(p, 0, 0);
        leftIO.configurePID(p, 0, 0);
        break;
      case REPLAY:
        p = 1;
        rightIO.configurePID(p, 0, 0);
        leftIO.configurePID(p, 0, 0);
        break;
    }

    leftIO.configurePID(1, 0, 0);
    rightIO.configurePID(1, 0, 0);
  }

  public void setVelocity(double leftVel, double rightVel) {
    leftIO.setVelocity(leftVel, 0);
    rightIO.setVelocity(rightVel, 0);
  }

  public void updateInputs() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);
  }
}
