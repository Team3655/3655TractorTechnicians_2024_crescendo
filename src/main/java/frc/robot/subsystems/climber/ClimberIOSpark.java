// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

/** Add your docs here. */
public class ClimberIOSpark implements ClimberIO {

  private final CANSparkMax rightMotor;
  private final RelativeEncoder rightEncoder;
  // most likely will not involve abs encoder
  // private final AbsoluteEncoder rightAbsolute;

  private final CANSparkMax leftMotor;
  private final RelativeEncoder leftEncoder;
  // private final AbsoluteEncoder leftAbsolute;

  public ClimberIOSpark() {

    rightMotor = new CANSparkMax(34, MotorType.kBrushless);
    rightEncoder = rightMotor.getEncoder();
    // rightAbsolute = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftMotor = new CANSparkMax(34, MotorType.kBrushless);
    leftEncoder = rightMotor.getEncoder();
    // leftAbsolute = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // will do this later
  }

  @Override
  public void setVoltage(double volts) {
    rightMotor.setVoltage(volts);
    leftMotor.setVoltage(volts);
  }

  // TODO: add set position method
}
