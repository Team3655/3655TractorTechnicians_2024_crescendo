// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climbing;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbingSubsystem extends SubsystemBase {
  private final ClimbingIO io;
  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem(ClimbingIO io) {
    this.io = io;

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
  }
}
