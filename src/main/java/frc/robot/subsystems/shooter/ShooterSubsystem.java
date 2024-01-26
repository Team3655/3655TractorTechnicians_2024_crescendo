// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  // a map that associates a distance to the target with an angle of the shooter
  private static final HashMap<Double, Rotation2d> DISTANCE_ANGLE_MAP =
      new HashMap<>() {
        {
          put(1.0, Rotation2d.fromDegrees(90));
          put(2.0, Rotation2d.fromDegrees(60));
          put(3.0, Rotation2d.fromDegrees(30));
          put(4.0, Rotation2d.fromDegrees(15));
          put(5.0, Rotation2d.fromDegrees(0));
        }
      };

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new shooter. */
  public ShooterSubsystem(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheels", inputs);
  }

  public void runVelocity(double leftVel, double rightVel) {
    io.setVelocity(leftVel, rightVel);
  }

  public void stop() {
    io.stop();
  }

  public void setShooterAngleFromDist() {}

  public Rotation2d getRotationFromDistance(double distance) {

    // use Doueble.(MAX/MIN)_VALUE as a way to ensure variables 
    // recieve a value from the map on the first loop
    double minimum = Double.MAX_VALUE;
    double minAbove = Double.MAX_VALUE;
    double maxBelow = Double.MIN_VALUE;
    double maximum = Double.MIN_VALUE;

    for (double distKey : DISTANCE_ANGLE_MAP.keySet()) {
      // sets the maxBelow if the key is greater than the current and less than the input
      if (distance > distKey && distKey > maxBelow) maxBelow = distKey;

      // sets the minAbove if the key is less than the current and greater than the input
      if (distance < distKey && distKey < minAbove) minAbove = distKey;

      // sets the minimum
      if (distKey < minimum) minimum = distKey;

      // sets the maximum
      if (distKey > maximum) maximum = distKey;
    }

    // return maximum value if distance is greater than the largest key in the map
    if (distance > maximum) 
      return DISTANCE_ANGLE_MAP.get(maximum);
    // return minimum value if distance is less than the smallest key in the map
    else if (distance < minimum)
      return DISTANCE_ANGLE_MAP.get(minimum);

    double lerpPercent = ((distance - maxBelow) / (minAbove - maxBelow));

    Rotation2d angleDelta =
        DISTANCE_ANGLE_MAP.get(minAbove).minus(DISTANCE_ANGLE_MAP.get(maxBelow));

    Rotation2d angle = DISTANCE_ANGLE_MAP.get(maxBelow).plus(angleDelta.times(lerpPercent));

    return angle;
  }
}
