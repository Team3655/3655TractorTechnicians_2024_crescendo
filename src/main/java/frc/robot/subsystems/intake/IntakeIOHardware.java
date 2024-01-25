// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeIOHardware implements IntakeIO {

  private final CANSparkMax suckerMotor = new CANSparkMax(31, MotorType.kBrushless);
  private final RelativeEncoder suckerEncoder = suckerMotor.getEncoder();
  private final AbsoluteEncoder deployEncoder =
      suckerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final Solenoid deploySolenoid = new Solenoid(50, PneumaticsModuleType.REVPH, 0);

  public IntakeIOHardware() {
    suckerMotor.restoreFactoryDefaults();
    suckerMotor.setCANTimeout(250);
    suckerMotor.enableVoltageCompensation(12.0);
    suckerMotor.setSmartCurrentLimit(30);
    suckerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            suckerEncoder.getVelocity() / IntakeSubsystem.SUCKER_GEAR_RATIO);
    inputs.intakeAppliedVolts = suckerMotor.getAppliedOutput() * suckerMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[] {suckerMotor.getOutputCurrent()};
    inputs.intakeMotorTemp = suckerMotor.getMotorTemperature();

    inputs.deploySolenoidState = deploySolenoid.get();
    inputs.deployPositionRads = Units.rotationsToRadians(deployEncoder.getPosition());
  }

  @Override
  public void setDeploy(boolean isDeployed) {
    deploySolenoid.set(isDeployed);
  }
}