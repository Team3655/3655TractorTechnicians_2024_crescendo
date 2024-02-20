package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;

/**
 * Implementation of the Swerve Module IO class. We use the Falcon500 motors to directly link the
 * software to the motors. For use with the SDS MK4i modules only.
 */
public class ModuleIOTalonFXPro implements ModuleIO {
  public static final int DRIVE_CURRENT_LIMIT = 40;
  public static final int SECONDARY_DRIVE_CURRENT_LIMIT = 30;
  public static final int STEER_CURRENT_LIMIT = 30;

  // Used to calculate feed forward for turn speed in 2nd order dynamics calc.
  public static final double TURN_kA = 0;
  public static final double TURN_kS = 0.05;
  public static final double TURN_kV = 0.1;

  // Constants specific to the hardware
  /** Radius of the wheel. Can be used to figure out distance data */
  private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);
  /** From motor rotations to the wheel revolutions */
  private static final double DRIVE_GEAR_RATIO = (50.0 / 18.0) * (16.0 / 28.0) * (45.0 / 15.0);

  /**
   * Conversion constant: From motor encoder ticks to position data (m)
   *
   * <p>motor encoder ticks -> meters: motor encoder ticks * constant meters -> motor encoder ticks:
   * meters / constant
   */
  private static final double DRIVE_SENSOR_POSITION_COEFFICIENT =
      (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;

  /**
   * Conversion constant: From motor encoder ticks to velocity data (m/s)
   *
   * <p>motor encoder ticks / 100 ms -> meters per second: motor encoder ticks * constant meters per
   * second -> motor encoder ticks / 100 ms: meters per second / constant
   */
  private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT = DRIVE_SENSOR_POSITION_COEFFICIENT;

  /**
   * From motor rotations to the wheel rotations (150 / 7 motor rotations : 1 full rotation of the
   * wheel [2π])
   */
  private static final double STEER_GEAR_RATIO = 150.0 / 7;
  /**
   * Conversion constant: From motor encoder ticks to angle data (steer position) (radians)
   *
   * <p>motor encoder ticks -> radians: motor encoder ticks * constant radians -> motor encoder
   * ticks: radians / constant
   */
  private static final double STEER_SENSOR_POSITION_COEFFICIENT =
      (2 * Math.PI) / (2048 * STEER_GEAR_RATIO);
  /**
   * Conversion constant: From motor encoder ticks to angular velocity data (steer velocity)
   * (radians/s)
   *
   * <p>motor encoder ticks / 100 ms -> radians per second: motor encoder ticks / 100 ms * constant
   * radians per second -> motor encoder ticks / 100 ms: radians per second / constant
   */
  private static final double STEER_SENSOR_VELOCITY_COEFFICIENT =
      STEER_SENSOR_POSITION_COEFFICIENT * 10;

  private static final double VELOCITY_COEFFICIENT = 1.8;

  // Hardware object initialization
  /** TalonFX swerve module drive motor */
  private final TalonFX driveMotor;
  /** TalonFX swerve module steer motor */
  private final TalonFX steerMotor;
  /** Swerve module steer encoder (absolute angular position) */
  private final CANcoder steerEncoder;

  // Target Variables. Used only for data logging
  private double targetVelocityMetersPerSeconds = 0;
  private double targetSteerAngleRadians = 0;
  private ArmFeedforward turnFFController;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final PositionVoltage steerPositionControl = new PositionVoltage(0.0);

  private StatusSignal<Double> primaryDrivePositionSignal;
  private StatusSignal<Double> primaryDriveVelocitySignal;
  private StatusSignal<Double> steerPositionSignal;
  private StatusSignal<Double> steerVelocitySignal;
  private BaseStatusSignal[] signals;

  /**
   * Initializes the motors, encoder, and the settings for each of the devices.
   *
   * @param driveMotorId Drive motor CAN ID
   * @param steerMotorId Steer motor CAN ID
   * @param steerEncoderId Steer encoder CAN ID
   * @param canBus The name of the CAN bus the device is connected to.
   * @param steerAngleOffsetRad This is the offset applied to the angle motor's absolute encoder so
   *     that a reading of 0 degrees means the module is facing forwards.
   */
  public ModuleIOTalonFXPro(
      int driveMotorId,
      int steerMotorId,
      int steerEncoderId,
      String canBus,
      double steerAngleOffsetRad) {

    driveMotor = new TalonFX(driveMotorId, canBus);
    steerMotor = new TalonFX(steerMotorId, canBus);

    steerEncoder = new CANcoder(steerEncoderId, canBus);

    FeedbackConfigs feedBackConfigs = new FeedbackConfigs();
    feedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveCurrentLimitsConfigs.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
    driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback = feedBackConfigs;
    driveConfig.CurrentLimits = driveCurrentLimitsConfigs;
    driveConfig.MotorOutput = motorOutputConfigs;
    driveConfig.Slot0.kP = 0.0;
    driveConfig.Slot0.kV =
        1023.0 / ((12.0 / VELOCITY_COEFFICIENT) / (DRIVE_SENSOR_VELOCITY_COEFFICIENT));

    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    //        steerConfig.
    //        steerConfig.primaryPID.selectedFeedbackSensor =
    // TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    //        steerConfig.slot0.kP = 0.2;
    //        steerConfig.supplyCurrLimit.currentLimit = STEER_CURRENT_LIMIT;
    //        steerConfig.supplyCurrLimit.enable = true;

    FeedbackConfigs feedBackSteerConfigs = new FeedbackConfigs();
    feedBackSteerConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    feedBackSteerConfigs.FeedbackRemoteSensorID = steerEncoderId;
    feedBackSteerConfigs.RotorToSensorRatio = STEER_GEAR_RATIO;

    CurrentLimitsConfigs steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerCurrentLimitsConfigs.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
    steerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 100;

    steerConfig.Feedback = feedBackSteerConfigs;
    steerConfig.CurrentLimits = steerCurrentLimitsConfigs;
    steerConfig.Slot0 = slot0Configs;

    MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
    steerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput = steerMotorOutputConfigs;

    steerMotor.getConfigurator().apply(steerConfig);

    CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();
    encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderMagnetSensorConfigs.MagnetOffset = Units.radiansToRotations(steerAngleOffsetRad);
    steerEncoderConfig.MagnetSensor = encoderMagnetSensorConfigs;

    steerEncoder.getConfigurator().apply(steerEncoderConfig);

    turnFFController = new ArmFeedforward(TURN_kS, 0.0, TURN_kV, TURN_kA);

    primaryDrivePositionSignal = driveMotor.getPosition();
    primaryDriveVelocitySignal = driveMotor.getVelocity();
    steerPositionSignal = steerEncoder.getPosition();
    steerVelocitySignal = steerEncoder.getVelocity();

    signals = new BaseStatusSignal[4];

    signals[0] = primaryDrivePositionSignal;
    signals[1] = primaryDriveVelocitySignal;
    signals[2] = steerPositionSignal;
    signals[3] = steerVelocitySignal;
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    double driveAppliedVolts = driveMotor.getSupplyVoltage().getValue();
    double steerAppliedVolts = steerMotor.getSupplyVoltage().getValue();

    inputs.drivePositionMeters =
        BaseStatusSignal.getLatencyCompensatedValue(
                primaryDrivePositionSignal, primaryDriveVelocitySignal)
            * (DRIVE_SENSOR_POSITION_COEFFICIENT);
    ;
    inputs.driveVelocityMetersPerSec =
        primaryDriveVelocitySignal.getValue() * (DRIVE_SENSOR_VELOCITY_COEFFICIENT);
    ;
    inputs.driveCurrentDrawAmps = driveMotor.getSupplyCurrent().getValue();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;

    inputs.steerPositionRad =
        Units.rotationsToRadians(
            BaseStatusSignal.getLatencyCompensatedValue(steerPositionSignal, steerVelocitySignal));
    inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
    inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocitySignal.getValue());
    inputs.steerCurrentDrawAmps = steerMotor.getSupplyCurrent().getValue();
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.targetSteerPositionRad = targetSteerAngleRadians;

    inputs.steerAbsolutePosition =
        Units.rotationsToRadians(steerEncoder.getAbsolutePosition().getValue());
  }

  @Override
  public void setTargetSteerPosition(double targetSteerPositionRad) {
    steerMotor.setControl(
        steerPositionControl
            .withPosition(Units.radiansToRotations(targetSteerPositionRad))
            .withFeedForward(0));
    this.targetSteerAngleRadians = targetSteerPositionRad;
  }

  @Override
  public void setTargetSteerAngle(double targetSteerAngleRadians, double turnSpeed) {
    final double turnArbFFComponent =
        turnFFController.calculate(targetSteerAngleRadians, turnSpeed);
    steerMotor.setControl(
        steerPositionControl
            .withPosition(Units.radiansToRotations(targetSteerAngleRadians))
            .withFeedForward(turnArbFFComponent));
    this.targetSteerAngleRadians = targetSteerAngleRadians;
  }

  @Override
  public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
    driveMotor.setControl(
        voltageControl.withOutput((targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12));

    Logger.recordOutput("Voltage", (targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12);

    this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
  }

  @Override
  public void resetToAbsoluteAngle() {}

  @Override
  public double getMaxVelocity() {
    return 12.0 / VELOCITY_COEFFICIENT;
  }

  @Override
  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}
