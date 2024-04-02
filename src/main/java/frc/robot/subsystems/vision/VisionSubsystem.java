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
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  /**
   * Multiply the distance to the vision target by this number in order to trust further
   * measurements less
   */
  private static final double ESTIMATION_COEFFICIENT = 0.65;

  private final VisionIO[] limelights;
  private final VisionIOInputsAutoLogged[] llInputs;

  private ArrayList<visionMeasurement> acceptedMeasurements = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO... limelights) {
    this.limelights = limelights;
    llInputs = new VisionIOInputsAutoLogged[limelights.length];
    for (int i = 0; i < limelights.length; i++) {
      llInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    acceptedMeasurements.clear();

    for (int i = 0; i < limelights.length; i++) {

      limelights[i].updateInputs(llInputs[i]);
      Logger.processInputs("Vision/" + limelights[i].getName(), llInputs[i]);

      for (int j = 0; j < llInputs[i].robotPose.length; j++) {

        if (llInputs[i].robotPose[j] == null) continue;
        if (llInputs[i].targetPoses.length < 1) continue;

        double xyStdDev =
            Math.pow(llInputs[i].distanceToCamera * ESTIMATION_COEFFICIENT, 1.6)
                / (double) llInputs[i].targetPoses.length;

        double thetaStdDev =
            Math.pow(llInputs[i].distanceToCamera * ESTIMATION_COEFFICIENT, 1.6)
                / (double) llInputs[i].targetPoses.length;

        if (llInputs[i].hasValidTarget && llInputs[i].distanceToCamera <= 6.0) {
          acceptedMeasurements.add(
              new visionMeasurement(
                  llInputs[i].robotPose[j],
                  llInputs[i].timestamp,
                  VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }
      }
    }
  }

  public ArrayList<visionMeasurement> getMeasurements() {
    return acceptedMeasurements;
  }

  /**
   * A class holding the data relevent to adding vision data to poseEstimation
   *
   * @param pose the Pose2d reported by the camera
   * @param distance the distance to the closest target
   * @param timestamp the timestamp of when the measurement was taken
   */
  public record visionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
