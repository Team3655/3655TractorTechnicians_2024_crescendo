// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTarget;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTargets;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterTarget targetState = ShooterTargets.IDLE;
  private Rotation2d pivotOffset = ShooterConstants.PIVOT_DEFAULT_OFFSET;

  /** Creates a new shooter. */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
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
    Logger.recordOutput("Shooter/PIVOT_DEFAULT_OFFSET", ShooterConstants.PIVOT_DEFAULT_OFFSET);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // modify the pivot target by the offset
    Rotation2d pivotTarget = targetState.angle().plus(pivotOffset);
    Logger.recordOutput("Shooter/Pivot Offset", pivotOffset);

    // ensure the target is within the safe operating range
    pivotTarget =
        Rotation2d.fromDegrees(
            Math.min(pivotTarget.getDegrees(), ShooterConstants.PIVOT_MAX_ANGLE.getDegrees()));
    pivotTarget =
        Rotation2d.fromDegrees(
            Math.max(pivotTarget.getDegrees(), ShooterConstants.PIVOT_MIN_ANGLE.getDegrees()));

    io.setAngle(pivotTarget, 0);
    Logger.recordOutput("Shooter/Pivot Target", pivotTarget);

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

  public void requestState(ShooterTarget state) {
    targetState = state;
  }

  public void setShooterAngleFromDist(double distance) {
    Rotation2d angle = Rotation2d.fromDegrees(ShooterConstants.ANGLE_MAP.get(distance));
    ShooterTarget state = new ShooterTarget(angle, Optional.of(5500.0));
    requestState(state);
  }

  public void jogAngleOffset(double degrees) {
    pivotOffset.plus(Rotation2d.fromDegrees(degrees));
  }
}
