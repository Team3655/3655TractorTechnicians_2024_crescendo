// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  
  private final ClimberIO io;
  // TODO: add autologged inputs

  /** Creates a new ClimbingSubsystem. */
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;

    // I don't think there will be a use for this be its fine to
    // leave ot for refrance until were sure 
    switch (Constants.currentMode) {
      case REAL:
        break;

      case SIM:
        break;

      case REPLAY:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: update inputs
  }

  // TODO: add set pose method
}
