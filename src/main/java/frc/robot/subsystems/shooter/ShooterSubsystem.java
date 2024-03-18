// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  public static final double KICKER_GEAR_RATIO = 5.0 / 1.0;

  // private static final HashMap<Double, Rotation2d> DISTANCE_TO_ANGLE =
  //     new HashMap<>() {
  //       {
  //         put(5.1436, Rotation2d.fromDegrees(63.7));
  //         put(4.9348, Rotation2d.fromDegrees(63.7));
  //         put(4.6436, Rotation2d.fromDegrees(62.0));
  //         put(4.5866, Rotation2d.fromDegrees(62.0));
  //         put(4.3000, Rotation2d.fromDegrees(62.0));
  //         put(4.0225, Rotation2d.fromDegrees(61.0));
  //         put(3.4130, Rotation2d.fromDegrees(57.0));
  //         put(2.7768, Rotation2d.fromDegrees(54.0));
  //         put(2.1333, Rotation2d.fromDegrees(45.0));
  //         put(1.6488, Rotation2d.fromDegrees(40.0));
  //         put(1.3854, Rotation2d.fromDegrees(36.0));
  //       }
  //     };

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState targetState = ShooterConstants.ShooterTargets.IDLE;

  /** Creates a new shooter. */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    // TODO: tune FF values for shooter
    switch (Constants.currentMode) {
      case REAL:
        io.configureFlywheelPID(0.001, 0.0, 0.0);
        io.configurePivotPID(30.0, 0.0, 0.0);
        break;

      case SIM:
        io.configureFlywheelPID(0.0, 0.0, 0.0);
        io.configurePivotPID(0.0, 0.0, 0.0);
        break;

      default:
        io.configureFlywheelPID(0.0, 0.0, 0.0);
        io.configurePivotPID(0.0, 0.0, 0.0);
        break;
    }

    // log gear ratios so that they can be inspected retroactivly
    Logger.recordOutput("Shooter/PIVOT_TRACK_RATIO ", ShooterConstants.PIVOT_TRACK_RATIO);
    Logger.recordOutput("Shooter/PIVOT_GEARBOX_RATIO", ShooterConstants.PIVOT_GEARBOX_RATIO);
    Logger.recordOutput("Shooter/PIVOT_GEAR_RATIO", ShooterConstants.PIVOT_GEAR_RATIO);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    io.setAngle(targetState.angle(), 0.0);
    Logger.recordOutput("Shooter/Pivot Target", targetState.angle());

    if (targetState.rpm().isPresent()) {
      io.setVelocity(targetState.rpm().get(), 0.0);
      Logger.recordOutput("Shooter/Target RPM", targetState.rpm().get());
    } else {
      io.stopFlywheel();
      Logger.recordOutput("Shooter/Target RPM", 0.0);
    }
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void requestState(ShooterState state) {
    targetState = state;
  }

  public void setShooterAngleFromDist(double distance) {
    Rotation2d angle = Rotation2d.fromDegrees(ShooterConstants.ANGLE_MAP.get(distance));
    ShooterState state = new ShooterState(angle, Optional.of(5000));
    requestState(state);
  }

  public void jogAngle(double degrees) {}

  // /**
  //  * Uses a distance input to the target to calculate a shooter angle
  //  *
  //  * @param distance the delta from target to robot
  //  * @return a Rotation2d represent the optimal shooter angle
  //  */
  // private Rotation2d getRotationFromDistance(double distance) {

  //   // Use max/min value to ensure variable gets a value on first loop cycle
  //   double minimum = Double.MAX_VALUE;
  //   double minAbove = Double.MAX_VALUE;
  //   double maxBelow = Double.MIN_VALUE;
  //   double maximum = Double.MIN_VALUE;

  //   for (double distKey : DISTANCE_TO_ANGLE.keySet()) {
  //     // finds the key key below the input
  //     if (distance > distKey && distKey > maxBelow) maxBelow = distKey;

  //     // finds the closest key above the input
  //     if (distance < distKey && distKey < minAbove) minAbove = distKey;

  //     // finds the maximum
  //     if (distKey < minimum) minimum = distKey;

  //     // finds the minimum
  //     if (distKey > maximum) maximum = distKey;
  //   }

  //   // if the input is outside the range of the map return
  //   // the minimum or maximum angle
  //   if (distance > maximum) return DISTANCE_TO_ANGLE.get(maximum);
  //   else if (distance < minimum) return DISTANCE_TO_ANGLE.get(minimum);

  //   // how close the input is to minAbove expressed as a percentage of the delta
  //   // between maxBelow and minAbove
  //   double lerpPercent = ((distance - maxBelow) / (minAbove - maxBelow));

  //   // the delta between the rotations associated with minAbove and minBelow
  //   Rotation2d angleDelta =
  // DISTANCE_TO_ANGLE.get(minAbove).minus(DISTANCE_TO_ANGLE.get(maxBelow));

  //   // The final target angle of the shooter uses the maxBelow angle as a starting
  //   // point,
  //   // then adds the angleDelta times the lerp percent
  //   Rotation2d angle = DISTANCE_TO_ANGLE.get(maxBelow).plus(angleDelta.times(lerpPercent));

  //   return angle;
  // }
}
