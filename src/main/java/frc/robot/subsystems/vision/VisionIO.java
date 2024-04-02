// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {
    public boolean hasValidTarget = false;
    public boolean isNew = false;

    public Pose3d[] targetPoses = new Pose3d[] {};
    public Pose2d[] robotPose = new Pose2d[] {};

    public double avgDistanceToCamera = 0.0;

    public double captureLatancySec = 0.0;
    public double pipelineLatancySec = 0.0;
    public double jsonParseLatancySec = 0.0;
    public double timestamp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Returns the name of the camera */
  public default String getName() {
    return "";
  }

  public default void updateRobotPose(Pose2d robotPose) {}
}
