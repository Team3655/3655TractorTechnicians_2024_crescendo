package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Connects the software to the hardware and directly receives data from the gyroscope. */
public interface GyroIO {
  /**
   * Reads information from sources (hardware or simulation) and updates the inputs object.
   *
   * @param inputs Contains the defaults for the input values listed above.
   */
  default void updateInputs(GyroIOInputs inputs) {}

  default BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[0];
  }

  /** Holds data that can be read from the corresponding gyroscope IO implementation. */
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();
    public double pitch = 0;
    public double roll = 0;
    public double yawVelocityRadPerSec = 0;
  }
}
