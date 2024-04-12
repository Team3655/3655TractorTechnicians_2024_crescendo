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
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Optional;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  private final String name;

  private Pose2d lastPose;

  public VisionIOLimelight(String name) {
    this.name = name;
    lastPose = new Pose2d();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    LimelightResults llresult = LimelightHelpers.getLatestResults(name);

    Pose2d robotPose = llresult.targetingResults.getBotPose2d_wpiBlue();
    inputs.isNew = !robotPose.equals(lastPose);

    int numTargets = llresult.targetingResults.targets_Fiducials.length;

    if (llresult.targetingResults.valid && numTargets > 0) {
      inputs.hasValidTarget = true;
      inputs.robotPose = new Pose2d[] {robotPose};

      // find the targets poses on the field
      inputs.targetPoses = new Pose3d[numTargets];
      for (int i = 0; i < numTargets; i++) {
        Optional<Pose3d> targetPose =
            TAG_LAYOUT.getTagPose((int) llresult.targetingResults.targets_Fiducials[i].fiducialID);
        inputs.targetPoses[i] = targetPose.get();
      }

      // calculate average distance from target to camera
      double totalDistance = 0.0;
      for (LimelightTarget_Fiducial target : llresult.targetingResults.targets_Fiducials) {
        totalDistance += target.getCameraPose_TargetSpace().getTranslation().getNorm();
      }
      inputs.avgDistanceToCamera = totalDistance / numTargets;

    } else {
      inputs.hasValidTarget = false;
      inputs.robotPose = new Pose2d[] {};
      inputs.targetPoses = new Pose3d[] {};
      inputs.avgDistanceToCamera = 0.0;
    }

    // log latency
    inputs.captureLatencySec =
        Units.millisecondsToSeconds(llresult.targetingResults.latency_capture);
    inputs.pipelineLatencySec =
        Units.millisecondsToSeconds(llresult.targetingResults.latency_pipeline);
    inputs.jsonParseLatencySec =
        Units.millisecondsToSeconds(llresult.targetingResults.latency_jsonParse);

    // record latency compensated timestamp
    inputs.timestamp =
        MathSharedStore.getTimestamp()
            - Units.millisecondsToSeconds(
                llresult.targetingResults.latency_pipeline
                    + llresult.targetingResults.latency_pipeline
                    + llresult.targetingResults.latency_jsonParse);

    lastPose = robotPose;
  }

  @Override
  public String getName() {
    return name;
  }
}
