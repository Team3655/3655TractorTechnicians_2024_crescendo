// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climbing;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

/** Add your docs here. */
public class ClimbingIOSpark implements ClimbingIO {

  private final CANSparkMax rightMotor;
  private final RelativeEncoder rightEncoder;
  private final AbsoluteEncoder rightAbsolute;

  private final CANSparkMax leftMotor;
  private final RelativeEncoder leftEncoder;
  private final AbsoluteEncoder leftAbsolute;

  public ClimbingIOSpark() {

    rightMotor = new CANSparkMax(34, MotorType.kBrushless);
    rightEncoder = rightMotor.getEncoder();
    rightAbsolute = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftMotor = new CANSparkMax(34, MotorType.kBrushless);
    leftEncoder = rightMotor.getEncoder();
    leftAbsolute = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void UpdateInputs(ClimbingIOInputs inputs) {
    // will do this later
  }

  @Override
  public void SetVoltage(double volts) {
    rightMotor.setVoltage(volts);
    leftMotor.setVoltage(volts);
  }
}
