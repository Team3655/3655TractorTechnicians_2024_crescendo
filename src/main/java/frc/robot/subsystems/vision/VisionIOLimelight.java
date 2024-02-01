// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private final String name;

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.deltaX = LimelightHelpers.getTX(name);
    inputs.robotPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
  }
}
