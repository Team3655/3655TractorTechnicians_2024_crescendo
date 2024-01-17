// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class ShooterSubsystem extends SubsystemBase {

  private final Flywheels flywheels;
  HashMap<Double, Rotation2d> distanceToAngleMap =
      new HashMap<>() {
        {
          put(1.0, Rotation2d.fromDegrees(90));
          put(2.0, Rotation2d.fromDegrees(60));
          put(3.0, Rotation2d.fromDegrees(30));
          put(4.0, Rotation2d.fromDegrees(15));
          put(5.0, Rotation2d.fromDegrees(0));
        }
      };

  /** Creates a new shooter. */
  public ShooterSubsystem(FlywheelIO leftIo, FlywheelIO rightIo) {
    flywheels = new Flywheels(leftIo, rightIo);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheels.periodic();
  }

  public void runVelocity(double leftVel, double rightVel) {
    flywheels.setVelocity(leftVel, rightVel);
  }

  public void stop() {
    flywheels.stop();
  }

  public void setShooterAngleFromDist() {}

  public Rotation2d getRotationFromDistance(double distance) {

    double minimum = Double.MAX_VALUE;
    double minAbove = Double.MAX_VALUE;
    double maxBelow = Double.MIN_VALUE;
    double maximum = Double.MIN_VALUE;

    for (double distKey : distanceToAngleMap.keySet()) {
      // Sets the minimum
      if (distance > distKey && distKey > maxBelow) maxBelow = distKey;

      // Sets the maximum
      if (distance < distKey && distKey < minAbove) minAbove = distKey;

      if (distKey < minimum) minimum = distKey;

      if (distKey > maximum) maximum = distKey;
    }

    double lerpPercent = ((distance - maxBelow) / (minAbove - maxBelow));

    Rotation2d angleDelta =
        distanceToAngleMap.get(minAbove).minus(distanceToAngleMap.get(maxBelow));

    Rotation2d angle = distanceToAngleMap.get(maxBelow).plus(angleDelta.times(lerpPercent));

    return angle;
  }
}
