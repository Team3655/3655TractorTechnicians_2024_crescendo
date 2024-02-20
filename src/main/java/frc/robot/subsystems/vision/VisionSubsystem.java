// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] inputs;

  private Pose2d robotPose = new Pose2d();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO limelight, VisionIO... cameras) {
    this.cameras = cameras;
    inputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateRobotPose(robotPose);
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + cameras[i].getName(), inputs[i]);
      Logger.recordOutput("Vision/Robot Pose", robotPose);
    }

    // Logger.recordOutput(
    //     "Vision/Limelight/Botpose Blue", LimelightHelpers.getBotPose_wpiBlue("limelight"));
  }

  public void updateRobotPose(Pose2d robotPose) {
    this.robotPose = robotPose;
  }

  // @AutoLogOutput(key = "Vision/Limelight/Botpose Blue")
  // public Pose2d getLLRobotPose() {
  //   double[] output = LimelightHelpers.getBotPose_wpiBlue("limelight");

  //   return new Pose2d(output[0], output[1], Rotation2d.fromDegrees(output[5]));
  // }

  // public boolean hasTarget() {
  //   if (LimelightHelpers.getTV("limelight")) return true;
  //   else return false;
  // }
}
