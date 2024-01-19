// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FlywheelIOSpark implements FlywheelIO {

  private static final double GEAR_RATIO = 1;

  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  public FlywheelIOSpark(int index) {

    // switch can id's and inverts based on index (index represents left vs right)
    switch (index) {
      case 1:
        motor = new CANSparkFlex(20, MotorType.kBrushless);
        motor.setInverted(false);
        break;
      case 2:
        motor = new CANSparkFlex(21, MotorType.kBrushless);
        motor.setInverted(true);
        break;

      default:
        throw new RuntimeException("Invalid flywheel index");
    }

    encoder = motor.getEncoder();
    pid = motor.getPIDController();

    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);
    motor.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.motorTemp = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
