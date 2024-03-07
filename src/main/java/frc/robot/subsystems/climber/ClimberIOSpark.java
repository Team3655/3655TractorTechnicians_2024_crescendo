// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ClimberIOSpark implements ClimberIO {

  private final CANSparkMax rightMotor;
  private final RelativeEncoder rightEncoder;

  private final CANSparkMax leftMotor;
  private final RelativeEncoder leftEncoder;

  private final SparkPIDController pid;

  public ClimberIOSpark(int rightID, int leftID) {

    rightMotor = new CANSparkMax(rightID, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setCANTimeout(250);
    rightMotor.enableVoltageCompensation(12.0);
    rightMotor.setSmartCurrentLimit(30);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightEncoder = rightMotor.getEncoder();

    leftMotor = new CANSparkMax(leftID, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setCANTimeout(250);
    rightMotor.enableVoltageCompensation(12.0);
    rightMotor.setSmartCurrentLimit(30);
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftEncoder = rightMotor.getEncoder();

    pid = rightMotor.getPIDController();
    pid.setOutputRange(-1.0, 1.0);
    pid.setP(0.06);

    leftMotor.follow(rightMotor, true);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftPositionRotations = leftEncoder.getPosition() / ClimberSubsystem.CLIMER_GEAR_RATIO;
    inputs.leftVelocityRPM = leftEncoder.getVelocity() / ClimberSubsystem.CLIMER_GEAR_RATIO;
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.leftMotorTemp = leftMotor.getMotorTemperature();

    inputs.rightPositionRotations = rightEncoder.getPosition() / ClimberSubsystem.CLIMER_GEAR_RATIO;
    inputs.rightVelocityRPM = rightEncoder.getVelocity() / ClimberSubsystem.CLIMER_GEAR_RATIO;
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightCurrentAmps = new double[] {rightMotor.getOutputCurrent()};
    inputs.rightMotorTemp = rightMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    rightMotor.setVoltage(volts);
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    pid.setReference(
        angle.getRotations() * ClimberSubsystem.CLIMER_GEAR_RATIO, ControlType.kPosition);
  }
}
