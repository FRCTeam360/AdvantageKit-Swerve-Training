package frc.robot.subsystems.launcher;

import frc.lib.mechanism.FlywheelMechanism;
import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MotorIO;

/** Launcher flywheel. Example of a {@link FlywheelMechanism}: goals are rotations per second. */
public class Flywheel extends FlywheelMechanism<Flywheel.Goal> {
  public enum Goal implements frc.lib.mechanism.Mechanism.Goal {
    SHOOT(50.0),
    PASS(35.0);

    private final double rps;

    Goal(double rps) {
      this.rps = rps;
    }

    @Override
    public double value() {
      return rps;
    }
  }

  public Flywheel(MechanismConfig config, MotorIO io) {
    super(config, io, Goal.class);
  }
}
