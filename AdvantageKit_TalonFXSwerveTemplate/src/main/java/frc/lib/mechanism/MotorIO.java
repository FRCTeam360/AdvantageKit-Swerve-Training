package frc.lib.mechanism;

import frc.lib.mechanism.MechanismConfig.Gains;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface shared by every motor-driven mechanism. All values are in the mechanism units
 * chosen in its {@link MechanismConfig}.
 *
 * <p>Every method has an empty default, so {@code new MotorIO() {}} is a valid IO for log replay or
 * for a mechanism the robot doesn't have.
 */
public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public boolean[] followersConnected = new boolean[0];
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(MotorIOInputs inputs) {}

  /** Open loop voltage. */
  public default void setVoltage(double volts) {}

  /** Closed loop position (Motion Magic if configured). */
  public default void setPosition(double position) {}

  /** Closed loop velocity in units/s. */
  public default void setVelocity(double velocity) {}

  /** Neutral output (brake or coast, as configured). */
  public default void stop() {}

  /** Tells the encoder the mechanism is at this position now. */
  public default void resetPosition(double position) {}

  public default void setGains(Gains gains) {}

  public default void setSoftLimitsEnabled(boolean enabled) {}
}
