package frc.robot.subsystems.arm;

import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.PositionMechanism;

/** Pivoting arm. Example of a {@link PositionMechanism}: goals are degrees, 0 = horizontal. */
public class Arm extends PositionMechanism<Arm.Goal> {
  public enum Goal implements frc.lib.mechanism.Mechanism.Goal {
    STOW(-15.0),
    INTAKE(5.0),
    SCORE(80.0);

    private final double degrees;

    Goal(double degrees) {
      this.degrees = degrees;
    }

    @Override
    public double value() {
      return degrees;
    }
  }

  public Arm(MechanismConfig config, MotorIO io) {
    super(config, io, Goal.class);
  }

  /** Example of a mechanism-specific helper: safe to drive under a low obstacle. */
  public boolean isLow() {
    return getPosition() < 20.0;
  }
}
