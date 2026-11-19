package frc.robot.subsystems.Launcher.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double voltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public double positionRotations = 0.0;
    public boolean connected = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public void setDutyCycle(double duty);

  public void setPosition(double position);
}
