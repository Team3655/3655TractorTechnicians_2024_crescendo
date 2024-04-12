// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private static final double TRANSLATION_COEFFICIENT = 0.55;
  private static final double ROTATION_COEFFICIENT = 2.0;

  // the maximum distance a measurement will be accepted in meters
  private static final double SINGLE_TAG_MAXIMUM = 4.5;
  private static final double MULTI_TAG_MAXIMUM = 6.5;

  private final VisionIO[] limelights;
  private final VisionIOInputsAutoLogged[] llInputs;

  private Queue<VisionMeasurement> acceptedMeasurements = new LinkedList<>();
  private ArrayList<Pose2d> acceptedPoses = new ArrayList<>();
  private ArrayList<Pose2d> rejectedPoses = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO... limelights) {
    this.limelights = limelights;
    // create and fill a list of autologged inputs;
    llInputs = new VisionIOInputsAutoLogged[limelights.length];
    for (int i = 0; i < limelights.length; i++) {
      llInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    rejectedPoses.clear();
    acceptedPoses.clear();

    for (int i = 0; i < limelights.length; i++) {

      limelights[i].updateInputs(llInputs[i]);
      Logger.processInputs("Vision/" + limelights[i].getName(), llInputs[i]);

      for (int j = 0; j < llInputs[i].robotPose.length; j++) {

        // basic checks to avoid errors
        if (!llInputs[i].isNew
            || !llInputs[i].hasValidTarget
            || llInputs[i].robotPose[j] == null
            || llInputs[i].targetPoses.length < 1) {
          continue;
        }

        // if the camera to target distance is too great reject the measurement
        // depending on the number of tags in view use a different maximum
        if (llInputs[i].avgDistanceToCamera
            >= (llInputs[i].targetPoses.length > 1 ? MULTI_TAG_MAXIMUM : SINGLE_TAG_MAXIMUM)) {
          // add pose to rejected poses for logging
          rejectedPoses.add(llInputs[i].robotPose[j]);
          continue;
        }

        // calculate the standard deviations for the robots x and y position
        double xyStdDev =
            Math.pow(llInputs[i].avgDistanceToCamera, 1.6)
                * TRANSLATION_COEFFICIENT
                / (double) llInputs[i].targetPoses.length;

        // calculate the standard deviations for the robots rotation
        double thetaStdDev =
            Math.pow(llInputs[i].avgDistanceToCamera, 1.6)
                * ROTATION_COEFFICIENT
                / (double) llInputs[i].targetPoses.length;

        // add pose to the accepted poses for logging
        acceptedPoses.add(llInputs[i].robotPose[j]);
        // add measurements to the queue
        acceptedMeasurements.add(
            new VisionMeasurement(
                llInputs[i].robotPose[j],
                llInputs[i].timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      }
    }

    Logger.recordOutput("Vision/acceptedPoses", acceptedPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("Vision/rejectedPoses", rejectedPoses.toArray(Pose2d[]::new));
  }
  /**
   * Gets the vision Measurements from the subsystem
   *
   * @return A Queue of VisionMeasurements
   */
  public Queue<VisionMeasurement> getMeasurements() {
    return acceptedMeasurements;
  }

  /**
   * A class holding the data relevant to adding vision data to poseEstimation
   *
   * @param pose the Pose2d reported by the camera
   * @param distance the distance to the closest target
   * @param timestamp the timestamp of when the measurement was taken
   */
  public record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
