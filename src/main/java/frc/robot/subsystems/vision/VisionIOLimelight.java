// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LimelightHelpers;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  private final String name;

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (LimelightHelpers.getTV(name)) {
      inputs.hasValidTarget = true;
      inputs.robotPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
      // inputs.distance = LimelightHelpers.getTargetPose_CameraSpace(name)
      inputs.targetPoses =
          new Pose3d[] {TAG_LAYOUT.getTagPose((int) LimelightHelpers.getFiducialID(name)).get()};
      inputs.timestamp =
          MathSharedStore.getTimestamp()
              - Units.millisecondsToSeconds(
                  LimelightHelpers.getLatency_Capture(name)
                      + LimelightHelpers.getLatency_Pipeline(name));
    } else {
      inputs.hasValidTarget = false;
    }
  }
}
