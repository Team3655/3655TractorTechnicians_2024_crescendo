// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] inputs;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(Supplier<Pose2d> robotPose, VisionIO... cameras) {
    this.cameras = cameras;
    inputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPoseSupplier(robotPose);
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" +cameras[i].getName(), inputs[i]);
    }
  }
}
