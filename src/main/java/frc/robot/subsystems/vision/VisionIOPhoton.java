// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// docs: https://docs.photonvision.org/en/latest/docs/simulation/simulation.html

/** Add your docs here. */
public class VisionIOPhoton implements VisionIO {

  // A vision system sim labelled as "main" in NetworkTables
  private static final VisionSystemSim SIM_SYSTEM = new VisionSystemSim("main");
  private final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
  private final PhotonPoseEstimator poseEstimator;
  private final PhotonCamera camera;

  private static final boolean isSim = Constants.currentMode == Mode.SIM;

  private Pose2d robotPose;

  /**
   * @param cameraName
   * @param trans
   * @throws IOException
   */
  public VisionIOPhoton(String cameraName, Transform3d trans) throws IOException {

    tagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    SIM_SYSTEM.addAprilTags(tagLayout);

    camera = new PhotonCamera(cameraName);

    poseEstimator =
        new PhotonPoseEstimator(
            tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            trans);
    poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    Logger.recordOutput("Vision/isSim", isSim);
    if (isSim) {
      SimCameraProperties cameraProperties = new SimCameraProperties();
      // A 640 x 480 camera with a 70 degree diagonal FOV.
      cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
      // Approximate detection noise with average and standard deviation error in
      // pixels.
      cameraProperties.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop
      // rate).
      cameraProperties.setFPS(50);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProperties.setAvgLatencyMs(14);
      cameraProperties.setLatencyStdDevMs(3);

      PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);

      // Add this camera to the vision system simulation with the given
      // robot-to-camera transform.
      SIM_SYSTEM.addCamera(cameraSim, trans);

      cameraSim.enableRawStream(true);
      cameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    if (isSim) SIM_SYSTEM.update(robotPose);

    // Gets the latest result and estimated pose from the camera
    PhotonPipelineResult result = camera.getLatestResult();
    Optional<EstimatedRobotPose> EstimatedRobotPose = poseEstimator.update(result);

    // Sets valid target to false if the estimator did not return a result or if the
    // camera collected an invalid tag ID
    if (EstimatedRobotPose.isPresent() && checkInvalidIDs(result.getTargets())) {
      // Basic inputs
      inputs.hasValidTarget = true;
      inputs.robotPose = EstimatedRobotPose.get().estimatedPose.toPose2d();
      inputs.timeStamp = EstimatedRobotPose.get().timestampSeconds;
      // List inputs
      inputs.ambiguity = new double[result.targets.size()];
      inputs.targetPoses = new Pose3d[result.targets.size()];
      // Fill list inputs
      for (int i = 0; i < result.targets.size(); i++) {
        inputs.ambiguity[i] = result.targets.get(i).getPoseAmbiguity();
        inputs.targetPoses[i] = tagLayout.getTagPose(result.targets.get(i).getFiducialId()).get();
      }
    } else {
      inputs.hasValidTarget = false;
    }
  }

  /**
   * Checks if all tag ID are less than the greatest tag ID on the field
   *
   * @param targets
   * @return
   */
  private boolean checkInvalidIDs(List<PhotonTrackedTarget> targets) {
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() > tagLayout.getTags().size() + 1)
        ;
      return false;
    }
    return true;
  }

  @Override
  public String getName() {
    return camera.getName();
  }

  @Override
  public void updateRobotPose(Pose2d robotPose) {
    this.robotPose = robotPose;
  }
}
