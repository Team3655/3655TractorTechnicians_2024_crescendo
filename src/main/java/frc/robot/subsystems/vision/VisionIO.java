// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {

    public boolean hasValidTarget = false;

    public double[] ambiguity = new double[] {};
    public Pose3d[] targetPoses = new Pose3d[] {};

    public Pose2d robotPose = new Pose2d();
    public double timeStamp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Returns the name of the camera */
  public default String getName() {
    return "";
  }

  public default void setPoseSupplier(Supplier<Pose2d> robotPose) {}

}
