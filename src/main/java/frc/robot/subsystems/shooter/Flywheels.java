// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Flywheels {

  private final FlywheelIO leftIO;
  private final FlywheelIO rightIO;
  private final FlywheelIOInputsAutoLogged leftInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged rightInputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  public Flywheels(FlywheelIO leftIO, FlywheelIO rightIO) {
    this.leftIO = leftIO;
    this.rightIO = rightIO;

    double p;
    switch (Constants.currentMode) {
      case REAL:
        p = 4;
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        rightIO.configurePID(p, 0, 0);
        leftIO.configurePID(p, 0, 0);
        break;
      case SIM:
        p = 1;
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        rightIO.configurePID(p, 0, 0);
        leftIO.configurePID(p, 0, 0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    leftIO.configurePID(1, 0, 0);
    rightIO.configurePID(1, 0, 0);
  }

  public void periodic() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);
    Logger.processInputs("Shooter/Left Flywheel", leftInputs);
    Logger.processInputs("Shooter/Right Flywheel", rightInputs);
  }

  public void setVelocity(double leftVelRPM, double rightVelRPM) {
    var leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftVelRPM);
    var rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightVelRPM);
    leftIO.setVelocity(leftVelocityRadPerSec, ffModel.calculate(leftVelocityRadPerSec));
    rightIO.setVelocity(rightVelocityRadPerSec, ffModel.calculate(rightVelocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/Left Flywheel/SetpointRPM", leftVelRPM);
    Logger.recordOutput("Shooter/Right Flywheel/SetpointRPM", rightVelRPM);
  }

  public void stop() {
    rightIO.stop();
    leftIO.stop();
  }
}
