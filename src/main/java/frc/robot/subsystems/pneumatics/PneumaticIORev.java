// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;

/** Add your docs here. */
public class PneumaticIORev implements PneumaticIO {

  private final PneumaticHub pneumaticHub;
  private final Compressor compressor;

  public PneumaticIORev() {
    pneumaticHub = new PneumaticHub(Pneumatics.PNEUMATIC_HUB_ID);
    compressor = pneumaticHub.makeCompressor();
  }

  @Override
  public void updateInputs(PneumaticIOInputs inputs) {
    inputs.pressurePSI = compressor.getPressure();
    inputs.compressorEnabled = compressor.isEnabled();
    inputs.compressorCurrent = compressor.getCurrent();
  }

  @Override
  public void enableWithThreshholds(double minimum, double maximum) {
    compressor.enableAnalog(minimum, maximum);
  }
}
