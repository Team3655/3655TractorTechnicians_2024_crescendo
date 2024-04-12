// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionConstants.VisionMode;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  private final Map<VisionMode, Integer> pipelines = new HashMap<>();

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

      inputs.targetErrorRads = Units.degreesToRadians(LimelightHelpers.getTX(name));

      // region: pose estimation
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
      // endregion

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

  @Override
  public void setMode(VisionMode mode) {
    if (!pipelines.containsKey(mode)) {
      System.out.println("WARNING, COULD NOT SET MODE! NO VALID PIPELINE FOUND FOR " + name.toUpperCase());
      return;
    }
    LimelightHelpers.setPipelineIndex(name, pipelines.get(mode));
  }

  /**
   * Adds a new association to VisionIOLimelight's pipeline mao and returns itself for method-chaining and easier to use.
   * <p>
   * The map cannot contain duplicate keys or indexes. if duplicates are entered a warning will be printed.
   *
   * @param mode A VisionMode to associate with and index
   * @param pipeline The index of the associate pipeline
   * @return
   */
  public VisionIOLimelight withPipeline(VisionMode mode, int pipeline) {
    if (pipelines.containsKey(mode) ) {
      System.out.println("WARNING, CANNOT ASSIGN PIPELINE FOR " + name.toUpperCase() + ": invalid key!");
    } else if (pipelines.containsValue(pipeline)) {
      System.out.println("WARNING, CANNOT ASSIGN PIPELINE FOR " + name.toUpperCase() + ": invalid pipeline ID!");
    } else {
      this.pipelines.put(mode, pipeline);
    }
    return this;
  }

}
