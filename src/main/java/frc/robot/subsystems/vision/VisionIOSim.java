// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

// docs: https://docs.photonvision.org/en/latest/docs/simulation/simulation.html

/** Add your docs here. */
public class VisionIOSim {

  // A vision system sim labelled as "main" in NetworkTables
  private static final VisionSystemSim SIM_SYSTEM = new VisionSystemSim("main");
  private static final boolean isSim =  Constants.currentMode == Constants.Mode.SIM;

  public VisionIOSim(String cameraName, Transform3d trans, Supplier<Pose2d> robotPose) throws IOException {

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    SIM_SYSTEM.addAprilTags(tagLayout);

    PhotonCamera camera = new PhotonCamera(cameraName);

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

  


}
