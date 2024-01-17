// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {
    public Pose2d pose = new Pose2d();
    public double timeStamp = 0.0;

    public double maxDistanceToTarget = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
