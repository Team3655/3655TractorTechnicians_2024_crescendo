// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class IntakeIOHardware implements IntakeIO {

  private final PneumaticHub pneumaticHub = new PneumaticHub(50);
  private final Compressor compressor = pneumaticHub.makeCompressor();
  private final Solenoid Linear = pneumaticHub.makeSolenoid(1);
  private final CANSparkMax suckerMotor = new CANSparkMax(40, MotorType.kBrushless);
  private final RelativeEncoder suckerEncoder = suckerMotor.getEncoder();
  private final DigitalInput proximity = new DigitalInput(0);

  public IntakeIOHardware() {
    suckerMotor.restoreFactoryDefaults();
    suckerMotor.setCANTimeout(250);
    suckerMotor.enableVoltageCompensation(12.0);
    suckerMotor.setSmartCurrentLimit(30);
    suckerMotor.burnFlash();

    Linear.set(false);
    pneumaticHub.enableCompressorAnalog(80, 120);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            suckerEncoder.getVelocity() / IntakeSubsystem.SUCKER_GEAR_RATIO);
    inputs.intakeAppliedVolts = suckerMotor.getAppliedOutput() * suckerMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[] {suckerMotor.getOutputCurrent()};
    inputs.intakeMotorTemp = suckerMotor.getMotorTemperature();

    inputs.hasPiece = !proximity.get();

    inputs.compressorPressure = compressor.getPressure();
    inputs.compressorCurrent = compressor.getCurrent();
    inputs.pneumaticHubVoltage = pneumaticHub.getInputVoltage();
    inputs.compressorEnabled = compressor.isEnabled();
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
  public boolean getProximity() {
    return !proximity.get();
  }

  @Override
  public void toggleLinear() {
    if (Linear.get()) {
      Linear.set(false);
    } else {
      Linear.set(true);
    }
  }

  @Override
  public double getPressure() {
    return pneumaticHub.getPressure(0);
  }
}
