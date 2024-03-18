// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;

  private final CANSparkMax feedMotor;
  private final RelativeEncoder feedEncoder;

  private final DigitalInput proximity;

  private final LaserCan indexDistance;

  public IntakeIODouble(
      int pneumaticHubID,
      int intakeMotorID,
      int indexMotorID,
      int indexDistanceID,
      int stageOneForwardPort,
      int stageOneReversePort,
      int stageTwoForwardPort,
      int stageTwoReversePort,
      int intakeBeamBreakPort) {

    pneumaticHub = new PneumaticHub(pneumaticHubID);
    compressor = pneumaticHub.makeCompressor();

    stageOne = pneumaticHub.makeDoubleSolenoid(stageOneForwardPort, stageOneReversePort);

    stageTwo = pneumaticHub.makeDoubleSolenoid(stageTwoForwardPort, stageTwoReversePort);

    pneumaticHub.enableCompressorAnalog(80, 120);

    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setCANTimeout(250);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(20);

    feedMotor = new CANSparkMax(indexMotorID, MotorType.kBrushless);
    feedEncoder = feedMotor.getEncoder();

    feedMotor.restoreFactoryDefaults();
    feedMotor.setCANTimeout(250);
    feedMotor.enableVoltageCompensation(12.0);
    feedMotor.setSmartCurrentLimit(20);
    feedMotor.setInverted(true);

    proximity = new DigitalInput(intakeBeamBreakPort);

    indexDistance = new LaserCan(indexDistanceID);
    try {
      indexDistance.setRangingMode(RangingMode.SHORT);
      indexDistance.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 14, 14));
      indexDistance.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCAN Configuration failed! " + e.getMessage());
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            intakeEncoder.getVelocity() / IntakeSubsystem.INTAKE_GEAR_RATIO);
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[] {intakeMotor.getOutputCurrent()};
    inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();

    inputs.feedVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            feedEncoder.getVelocity() / IntakeSubsystem.FEEDER_GEAR_RATIO);
    inputs.feedAppliedVolts = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
    inputs.feedCurrentAmps = new double[] {feedMotor.getOutputCurrent()};
    inputs.feedMotorTemp = feedMotor.getMotorTemperature();

    inputs.indexDistanceMM = indexDistance.getMeasurement().distance_mm;
    inputs.indexDistanceAmbient = indexDistance.getMeasurement().ambient;

    inputs.hasPiece = !proximity.get();

    inputs.compressorPressure = compressor.getPressure();
    inputs.compressorCurrent = compressor.getCurrent();
    inputs.pneumaticHubVoltage = pneumaticHub.getInputVoltage();
    inputs.compressorEnabled = compressor.isEnabled();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feedMotor.setVoltage(volts);
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
