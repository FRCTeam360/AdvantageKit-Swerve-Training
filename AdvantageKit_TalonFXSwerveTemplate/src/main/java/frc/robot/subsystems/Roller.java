package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  // FIX ME !!!! THE THREEEEEEE
  private final TalonFX motor = new TalonFX(3);

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void stop() {
    motor.stopMotor();
  }
}
