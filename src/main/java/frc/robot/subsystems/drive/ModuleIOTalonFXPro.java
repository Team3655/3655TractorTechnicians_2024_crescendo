package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
  private static final int DRIVE_CURRENT_LIMIT = 20;
  private static final int STEER_CURRENT_LIMIT = 15;

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
   * <p>rotations -> meters
   */
  private static final double DRIVE_SENSOR_POSITION_COEFFICIENT =
      (2 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;

  /**
   * Conversion constant: From motor encoder ticks to velocity data (m/s)
   *
   * <p>rotations / second -> meters per second
   */
  private static final double DRIVE_SENSOR_VELOCITY_COEFFICIENT = DRIVE_SENSOR_POSITION_COEFFICIENT;

  /**
   * From motor rotations to the wheel rotations (150 / 7 motor rotations : 1 full rotation of the
   * wheel [2π])
   */
  private static final double STEER_GEAR_RATIO = 150.0 / 7;

  private static final double MAX_VELOCITY = Units.feetToMeters(17.1);

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
   * @param absoluteOffsetRotations This is the offset applied to the angle motor's absolute encoder
   *     so that a reading of 0 degrees means the module is facing forwards.
   */
  public ModuleIOTalonFXPro(
      int driveMotorId,
      int steerMotorId,
      int steerEncoderId,
      String canBus,
      double absoluteOffsetRotations) {

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
    driveConfig.Slot0.kP = 0.3;
    driveConfig.Slot0.kV = 0.13;

    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration steerConfig = new TalonFXConfiguration();

    FeedbackConfigs feedBackSteerConfigs = new FeedbackConfigs();
    feedBackSteerConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    feedBackSteerConfigs.FeedbackRemoteSensorID = steerEncoderId;
    feedBackSteerConfigs.RotorToSensorRatio = STEER_GEAR_RATIO;

    CurrentLimitsConfigs steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerCurrentLimitsConfigs.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
    steerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    steerConfig.Feedback = feedBackSteerConfigs;
    steerConfig.CurrentLimits = steerCurrentLimitsConfigs;
    steerConfig.Slot0.kP = 100.0;

    MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
    steerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    steerConfig.MotorOutput = steerMotorOutputConfigs;

    steerMotor.getConfigurator().apply(steerConfig);

    CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
    MagnetSensorConfigs encoderMagnetSensorConfigs = new MagnetSensorConfigs();
    encoderMagnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderMagnetSensorConfigs.MagnetOffset = absoluteOffsetRotations;
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

    inputs.driveVelocityMetersPerSec =
        primaryDriveVelocitySignal.getValue() * (DRIVE_SENSOR_VELOCITY_COEFFICIENT);

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
        targetDriveVelocityMetersPerSec != 0.0
            ? velocityControl.withVelocity(
                targetDriveVelocityMetersPerSec / DRIVE_SENSOR_VELOCITY_COEFFICIENT)
            : voltageControl.withOutput(
                (targetDriveVelocityMetersPerSec / getMaxVelocity()) * 12.0));

    this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
  }

  @Override
  public void resetToAbsoluteAngle() {}

  @Override
  public double getMaxVelocity() {
    return MAX_VELOCITY;
  }

  @Override
  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}
