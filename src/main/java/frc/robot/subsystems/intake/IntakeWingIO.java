// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeWingIO {

  @AutoLog
  public static class IntakeWingIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

}