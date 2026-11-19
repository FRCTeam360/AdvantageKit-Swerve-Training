package frc.robot.subsystems.Launcher.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs(getName(), inputs);
  }
}
