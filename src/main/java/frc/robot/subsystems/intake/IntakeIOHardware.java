// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */
public class IntakeIOHardware implements IntakeIO {

  private final CANSparkMax suckerMotor = new CANSparkMax(40, MotorType.kBrushless);
  private final RelativeEncoder suckerEncoder = suckerMotor.getEncoder();
  private final DoubleSolenoid LinearRight =
      new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 0, 1);
  private final DoubleSolenoid LinearLeft =
      new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 2, 3);
  private final DoubleSolenoid RotateRight =
      new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 5);
  private final DoubleSolenoid RotateLeft =
      new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 6, 7);

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
  }

  @Override
  public void setVoltage(double volts) {
    suckerMotor.setVoltage(volts);
  }

  @Override
  public void toggle() {
    LinearRight.toggle();
    LinearLeft.toggle();
    RotateRight.toggle();
    RotateLeft.toggle();
  }

  @Override
  public void setLinear(Value value) {
    LinearRight.set(value);
    LinearLeft.set(value);
  }

  @Override
  public void setRotate(Value value) {
    RotateLeft.set(value);
    RotateRight.set(value);
  }
}
