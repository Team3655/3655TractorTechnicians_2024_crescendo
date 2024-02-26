// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class MirroredPose {

  private final Pose2d bluePose;

  public MirroredPose(Pose2d bluePose) {
    this.bluePose = bluePose;
  }

  public Pose2d getPose() {
    return bluePose;
  }
}
