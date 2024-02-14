// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PneumaticIO {

  @AutoLog
  public static class PneumaticIOInputs {
    public double pressurePSI = 0.0;
    public boolean compressorEnabled = false;
    public double compressorCurrent = 0.0;
  }

  public default void updateInputs(PneumaticIOInputs inputs) {}

  public default void enableWithThreshholds(double minimum, double maximum) {}
}
