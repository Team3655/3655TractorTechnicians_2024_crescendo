package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(20, Constants.RIO_BUS_NAME);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZDevice();
  private final StatusSignal<Double> xAcceleration = pigeon.getAccelerationX();
  private final StatusSignal<Double> yAcceleration = pigeon.getAccelerationY();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    xAcceleration.setUpdateFrequency(50.0);
    yAcceleration.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    if (phoenixDrive) {
      yawPositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    } else {
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(() -> pigeon.getYaw().getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, xAcceleration, yAcceleration)
            .equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.xAccelerationMetersPerSec = xAcceleration.getValueAsDouble();
    inputs.yAccelerationMetersPerSec = yAcceleration.getValueAsDouble();

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
