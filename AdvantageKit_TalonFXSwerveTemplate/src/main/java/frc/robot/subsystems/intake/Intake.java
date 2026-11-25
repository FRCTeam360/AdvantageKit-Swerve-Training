package frc.robot.subsystems.intake;

import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.RollerMechanism;

/** Intake roller. Example of a {@link RollerMechanism}: goals are volts. */
public class Intake extends RollerMechanism<Intake.Goal> {
  public enum Goal implements frc.lib.mechanism.Mechanism.Goal {
    INTAKE(8.0),
    EJECT(-6.0);

    private final double volts;

    Goal(double volts) {
      this.volts = volts;
    }

    @Override
    public double value() {
      return volts;
    }
  }

  public Intake(MechanismConfig config, MotorIO io) {
    super(config, io, Goal.class);
  }
}
