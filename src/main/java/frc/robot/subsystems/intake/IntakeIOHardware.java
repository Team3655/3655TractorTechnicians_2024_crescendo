// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeIOHardware implements IntakeIO {

  private final PneumaticHub pneumaticHub = new PneumaticHub(50);
  private final Solenoid Linear = pneumaticHub.makeSolenoid(1);
  private final CANSparkMax suckerMotor = new CANSparkMax(40, MotorType.kBrushless);
  private final RelativeEncoder suckerEncoder = suckerMotor.getEncoder();
  // private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  // private final Solenoid Linear = new Solenoid(PneumaticsModuleType.REVPH, 1);
  // private final DoubleSolenoid LinearLeft =
  //     new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 2, 3);
  // private final DoubleSolenoid RotateRight =
  //     new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 5);
  // private final DoubleSolenoid RotateLeft =
  //     new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 6, 7);

  public IntakeIOHardware() {
    suckerMotor.restoreFactoryDefaults();
    suckerMotor.setCANTimeout(250);
    suckerMotor.enableVoltageCompensation(12.0);
    suckerMotor.setSmartCurrentLimit(30);
    suckerMotor.burnFlash();

    Linear.set(false);
    pneumaticHub.enableCompressorAnalog(50, 120);
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
  public void setLinear(Boolean value) {
    Linear.set(value);
  }

  @Override
  public void suckUpNote(Boolean value, int volts) {
    suckerMotor.setVoltage(volts);
    Linear.set(value);
  }
}
