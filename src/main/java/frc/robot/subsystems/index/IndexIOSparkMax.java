// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IndexIOSparkMax implements IndexIO {

  private static final int INDEX_CURRENT_LIMIT = 12;
  private static final int PIVOT_CURRENT_LIMIT = 20;

  private final CANSparkMax indexMotor = new CANSparkMax(50, MotorType.kBrushless);
  private final RelativeEncoder indexEncoder = indexMotor.getEncoder();

  private final CANSparkMax pivotMotor = new CANSparkMax(51, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  public IndexIOSparkMax() {
    indexMotor.restoreFactoryDefaults();
    indexMotor.setCANTimeout(250);
    indexMotor.enableVoltageCompensation(12.0);
    indexMotor.setSmartCurrentLimit(INDEX_CURRENT_LIMIT);
    indexMotor.burnFlash();

    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setCANTimeout(250);
    pivotMotor.enableVoltageCompensation(12.0);
    pivotMotor.setSmartCurrentLimit(PIVOT_CURRENT_LIMIT);
    pivotMotor.burnFlash();
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.indexPositionRads = indexEncoder.getPosition();
    inputs.indexVelRadsPerSec = Units.rotationsToRadians(indexEncoder.getVelocity());
    inputs.indexAppliedVolts = indexMotor.getAppliedOutput() * indexMotor.getBusVoltage();
    inputs.indexCurrentAmps = new double[] {indexMotor.getOutputCurrent()};
    inputs.indexMotorTemp = indexMotor.getMotorTemperature();

    inputs.pivotPosition = Rotation2d.fromRotations(pivotEncoder.getPosition());
    inputs.pivotVelRadsPerSec = pivotEncoder.getVelocity();
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.indexCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.pivotMotorTemp = pivotMotor.getMotorTemperature();
  }
}
