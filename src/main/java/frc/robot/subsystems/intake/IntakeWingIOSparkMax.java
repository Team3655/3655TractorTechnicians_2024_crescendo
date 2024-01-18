// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakeWingIOSparkMax implements IntakeWingIO {

  private final CANSparkMax suckerMotor = new CANSparkMax(31, MotorType.kBrushless);
  private final RelativeEncoder suckerEncoder = suckerMotor.getEncoder();

  private final CANSparkMax deployMotor = new CANSparkMax(32, MotorType.kBrushless);
  private final RelativeEncoder deployEncoder = deployMotor.getEncoder();
  private final SparkPIDController deployPID = deployMotor.getPIDController();

  public IntakeWingIOSparkMax() {

    suckerMotor.restoreFactoryDefaults();
    suckerMotor.setCANTimeout(250);
    suckerMotor.enableVoltageCompensation(12.0);
    suckerMotor.setSmartCurrentLimit(30);
    suckerMotor.burnFlash();

    deployMotor.restoreFactoryDefaults();
    deployMotor.setCANTimeout(250);
    deployMotor.enableVoltageCompensation(12.0);
    deployMotor.setSmartCurrentLimit(30);
    deployMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeWingIOInputs inputs) {
    inputs.intakeAngularVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            suckerEncoder.getVelocity() / IntakeSubsystem.SUCKER_GEAR_RATIO);
    inputs.intakeAppliedVolts = suckerMotor.getAppliedOutput() * suckerMotor.getBusVoltage();
    inputs.intakeCurrentAmps = suckerMotor.getOutputCurrent();
    inputs.intakeMotorTemp = suckerMotor.getMotorTemperature();

    inputs.deployPositionRads =
        Units.rotationsToRadians(deployEncoder.getPosition() / IntakeSubsystem.DEPLOY_GEAR_RATIO);
    inputs.deployAppliedVolts = deployMotor.getAppliedOutput() * deployMotor.getBusVoltage();
    inputs.deployCurrentAmps = deployMotor.getOutputCurrent();
    inputs.deployMotorTemp = deployMotor.getMotorTemperature();
  }

  @Override
  public void configureDeployPID(double p, double i, double d) {
    deployPID.setP(p);
    deployPID.setI(i);
    deployPID.setD(d);
  }
}
