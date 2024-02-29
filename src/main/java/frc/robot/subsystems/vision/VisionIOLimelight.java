// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;

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
      inputs.robotPose = new Pose2d[] {LimelightHelpers.getBotPose2d_wpiBlue(name)};
      inputs.distanceToCamera =
          LimelightHelpers.getCameraPose3d_TargetSpace(name).getTranslation().getNorm();
      Optional<Pose3d> targetPose =
          TAG_LAYOUT.getTagPose((int) LimelightHelpers.getFiducialID(name));
      if (targetPose.isPresent()) {
        inputs.targetPoses = new Pose3d[] {targetPose.get()};
      }
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
