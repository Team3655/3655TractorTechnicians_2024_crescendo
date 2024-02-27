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

  private final PneumaticHub pneumaticHub;
  private final Compressor compressor;
  private final Solenoid extension;
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final DigitalInput proximity;

  public IntakeIOHardware(
      int pneumaticHubID, int intakeMotorID, int deploySolenoidPort, int intakeBeamBreakPort) {

    pneumaticHub = new PneumaticHub(pneumaticHubID);
    compressor = pneumaticHub.makeCompressor();
    extension = pneumaticHub.makeSolenoid(deploySolenoidPort);

    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    proximity = new DigitalInput(intakeBeamBreakPort);

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setCANTimeout(250);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.burnFlash();

    extension.set(false);
    pneumaticHub.enableCompressorAnalog(80, 120);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            intakeEncoder.getVelocity() / IntakeSubsystem.SUCKER_GEAR_RATIO);
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[] {intakeMotor.getOutputCurrent()};
    inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();

    inputs.hasPiece = !proximity.get();

    inputs.compressorPressure = compressor.getPressure();
    inputs.compressorCurrent = compressor.getCurrent();
    inputs.pneumaticHubVoltage = pneumaticHub.getInputVoltage();
    inputs.compressorEnabled = compressor.isEnabled();
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setLinear(Boolean value) {
    extension.set(value);
  }

  @Override
  public boolean getProximity() {
    return !proximity.get();
  }

  @Override
  public void toggleLinear() {
    if (extension.get()) {
      extension.set(false);
    } else {
      extension.set(true);
    }
  }

  @Override
  public double getPressure() {
    return pneumaticHub.getPressure(0);
  }
}
