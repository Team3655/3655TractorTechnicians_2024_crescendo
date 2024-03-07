// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO limelight;
  private final VisionIOInputsAutoLogged llInputs = new VisionIOInputsAutoLogged();

  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] inputs;

  private Pose2d robotPose = new Pose2d();

  private ArrayList<visionMeasurement> acceptedMeasurements = new ArrayList<>();
  private ArrayList<visionMeasurement> rejectedMeasurements = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO limelight, VisionIO... cameras) {
    this.limelight = limelight;
    this.cameras = cameras;
    inputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    acceptedMeasurements.clear();

    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateRobotPose(robotPose);
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + cameras[i].getName(), inputs[i]);
    }

    limelight.updateInputs(llInputs);
    Logger.processInputs("Vision/limelight", llInputs);

    for (int i = 0; i < llInputs.robotPose.length; i++) {
      if (llInputs.hasValidTarget && llInputs.distanceToCamera <= 5.0) {
        acceptedMeasurements.add(
            new visionMeasurement(
                llInputs.robotPose[i], llInputs.distanceToCamera, llInputs.timestamp));
      } else {
        rejectedMeasurements.add(
            new visionMeasurement(
                llInputs.robotPose[i], llInputs.distanceToCamera, llInputs.timestamp));
      }
    }
  }

  public ArrayList<visionMeasurement> getMeasurements() {
    return acceptedMeasurements;
  }

  public void updateRobotPose(Pose2d robotPose) {
    this.robotPose = robotPose;
  }

  public class visionMeasurement {
    private final Pose2d pose;
    private final double timestamp;
    private final double distance;

    /**
     * A class holding the data relevent to adding vision data to poseEstimation
     *
     * @param pose the Pose2d reported by the camera
     * @param distance the distance to the closest target
     * @param timestamp the timestamp of when the measurement was taken
     */
    public visionMeasurement(Pose2d pose, double distance, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.distance = distance;
    }

    /**
     * @return the Pose2d reported by the camera
     */
    public Pose2d getPose() {
      return pose;
    }

    /**
     * @return the distance to the closest target
     */
    public double getDistance() {
      return distance;
    }

    /**
     * @return the timestamp of when the measurement was taken
     */
    public double getTimestamp() {
      return timestamp;
    }
  }
}
