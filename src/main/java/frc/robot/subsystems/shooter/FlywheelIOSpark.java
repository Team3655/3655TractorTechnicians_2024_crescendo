// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FlywheelIOSpark implements FlywheelIO {

  private static final double GEAR_RATIO = 1;

  // private final CANSparkFlex top;
  // private final RelativeEncoder topEncoder;
  // private final SparkPIDController topPID;

  private final CANSparkFlex bottom;
  private final RelativeEncoder bottomEncoder;
  private final SparkPIDController bottomPID;

  private final CANSparkMax kicker;
  private final RelativeEncoder kickerEncoder;

  public FlywheelIOSpark() {

    // top flywheel
    // top = new CANSparkFlex(60, MotorType.kBrushless);

    // top.restoreFactoryDefaults();
    // top.setCANTimeout(250);
    // top.enableVoltageCompensation(12.0);
    // top.setSmartCurrentLimit(30);
    // top.burnFlash();

    // topEncoder = top.getEncoder();
    // topPID = top.getPIDController();

    // bottom flywheel
    bottom = new CANSparkFlex(31, MotorType.kBrushless);

    bottom.restoreFactoryDefaults();
    bottom.setCANTimeout(250);
    bottom.enableVoltageCompensation(12.0);
    bottom.setSmartCurrentLimit(30);
    bottom.burnFlash();

    bottomEncoder = bottom.getEncoder();
    bottomPID = bottom.getPIDController();

    // kicker
    kicker = new CANSparkMax(32, MotorType.kBrushless);

    kicker.restoreFactoryDefaults();
    kicker.setCANTimeout(250);
    kicker.enableVoltageCompensation(12.0);
    kicker.setSmartCurrentLimit(20);
    kicker.burnFlash();

    kickerEncoder = kicker.getEncoder();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // inputs.topPositionRad = Units.rotationsToRadians(topEncoder.getPosition() / GEAR_RATIO);
    // inputs.topVelocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(topEncoder.getVelocity() / GEAR_RATIO);
    // inputs.topAppliedVolts = top.getAppliedOutput() * top.getBusVoltage();
    // inputs.topCurrentAmps = new double[] {top.getOutputCurrent()};
    // inputs.topMotorTemp = top.getMotorTemperature();

    inputs.bottomPositionRad = Units.rotationsToRadians(bottomEncoder.getPosition() / GEAR_RATIO);
    inputs.bottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomEncoder.getVelocity() / GEAR_RATIO);
    inputs.bottomAppliedVolts = bottom.getAppliedOutput() * bottom.getBusVoltage();
    inputs.bottomCurrentAmps = new double[] {bottom.getOutputCurrent()};
    inputs.bottomMotorTemp = bottom.getMotorTemperature();

    inputs.kickerPositionRad = Units.rotationsToRadians(kickerEncoder.getPosition() / GEAR_RATIO);
    inputs.kickerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(kickerEncoder.getVelocity() / GEAR_RATIO);
    inputs.kickerAppliedVolts = kicker.getAppliedOutput() * kicker.getBusVoltage();
    inputs.kickerCurrentAmps = new double[] {kicker.getOutputCurrent()};
    inputs.kickerMotorTemp = kicker.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    // top.setVoltage(volts);
    bottom.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // topPID.setReference(
    //     Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
    //     ControlType.kVelocity,
    //     0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    // top.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // top PID
    // topPID.setP(kP, 0);
    // topPID.setI(kI, 0);
    // topPID.setD(kD, 0);
    // topPID.setFF(0, 0);
    // bottom PID
    bottomPID.setP(kP, 0);
    bottomPID.setI(kI, 0);
    bottomPID.setD(kD, 0);
    bottomPID.setFF(0, 0);
  }
}
