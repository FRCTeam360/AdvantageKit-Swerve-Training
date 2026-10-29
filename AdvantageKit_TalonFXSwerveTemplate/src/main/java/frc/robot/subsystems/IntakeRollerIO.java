package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  /** Creates a new IntakeRollerIO. */
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double rollerPosition = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerDutyCycle = 0.0;
    public double rollerCurrent = 0.0;
  }

  public void setDutyCycle(double dutyCycle);

  public void stop();

  public void setVelocity(double rpm);

  public default void updateInputs(IntakeRollerIOInputs inputs) {}
}
