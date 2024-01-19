// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;
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

  private Supplier<Pose2d> robotPose;

  public VisionIOPhoton(String cameraName, Transform3d trans, Supplier<Pose2d> robotPose)
      throws IOException {

    this.robotPose = robotPose;

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

      cameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (isSim) SIM_SYSTEM.update(robotPose.get());

    PhotonPipelineResult result = camera.getLatestResult();
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
}
