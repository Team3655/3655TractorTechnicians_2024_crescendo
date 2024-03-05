// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;

/** Add your docs here. */
public class IntakeIODouble implements IntakeIO {

  private final PneumaticHub pneumaticHub;
  private final Compressor compressor;
  private final DoubleSolenoid stageOne;
  private final DoubleSolenoid stageTwo;
  private final CANSparkFlex intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final DigitalInput proximity;

  public IntakeIODouble(
      int pneumaticHubID,
      int intakeMotorID,
      int stageOneForwardPort,
      int stageOneReversePort,
      int stageTwoForwardPort,
      int stageTwoReversePort,
      int intakeBeamBreakPort) {

    pneumaticHub = new PneumaticHub(pneumaticHubID);
    compressor = pneumaticHub.makeCompressor();

    stageOne = pneumaticHub.makeDoubleSolenoid(stageOneForwardPort, stageOneReversePort);

    stageTwo = pneumaticHub.makeDoubleSolenoid(stageTwoForwardPort, stageTwoReversePort);

    intakeMotor = new CANSparkFlex(intakeMotorID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    proximity = new DigitalInput(intakeBeamBreakPort);

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setCANTimeout(250);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(30);

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
  public void setDeployed(Boolean value) {
    if (value) {
      stageOne.set(Value.kForward);
      stageTwo.set(Value.kForward);
    } else {
      stageOne.set(Value.kReverse);
      stageTwo.set(Value.kReverse);
    }
  }

  @Override
  public void setStageOneValue(Value value) {
    stageOne.set(value);
  }

  @Override
  public void setStageTwoValue(Value value) {
    stageTwo.set(value);
  }

  @Override
  public boolean getProximity() {
    return !proximity.get();
  }

  @Override
  public double getPressure() {
    return pneumaticHub.getPressure(0);
  }
}
