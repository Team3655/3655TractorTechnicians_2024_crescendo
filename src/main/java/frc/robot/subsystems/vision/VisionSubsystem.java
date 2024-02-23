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

  private ArrayList<TimestampedPose> acceptedMeasurements = new ArrayList<>();
  private ArrayList<TimestampedPose> rejectedMeasurements = new ArrayList<>();

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
    if (llInputs.hasValidTarget) {
      acceptedMeasurements.add(new TimestampedPose(llInputs.robotPose, llInputs.timestamp));
    }
  }

  public ArrayList<TimestampedPose> getMeasurements() {
    return acceptedMeasurements;
  }

  public void updateRobotPose(Pose2d robotPose) {
    this.robotPose = robotPose;
  }

  public class TimestampedPose {
    private final Pose2d pose;
    private final double timestamp;

    public TimestampedPose(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }

    public Pose2d getPose() {
      return pose;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }
}
