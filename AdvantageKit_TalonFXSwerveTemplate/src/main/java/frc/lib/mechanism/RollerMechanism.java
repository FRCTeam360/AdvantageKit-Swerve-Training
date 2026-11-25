package frc.lib.mechanism;

/**
 * A mechanism run by voltage: intakes, indexers, conveyors. Goal values are volts (-12 to 12).
 *
 * <pre>{@code
 * public class Intake extends RollerMechanism<Intake.Goal> {
 *   public enum Goal implements Mechanism.Goal {
 *     INTAKE(8.0), EJECT(-6.0);
 *     private final double volts;
 *     Goal(double volts) { this.volts = volts; }
 *     public double value() { return volts; }
 *   }
 *   public Intake(MechanismConfig config, MotorIO io) { super(config, io, Goal.class); }
 * }
 * }</pre>
 */
public abstract class RollerMechanism<G extends Enum<G> & Mechanism.Goal> extends Mechanism<G> {
  protected RollerMechanism(MechanismConfig config, MotorIO io, Class<G> goalType) {
    super(config, io, goalType);
  }

  @Override
  protected void applySetpoint(double volts) {
    io.setVoltage(volts);
  }

  @Override
  protected void applyIdle() {
    io.stop();
  }

  /** Voltage is applied immediately, so a roller is always at its goal. */
  @Override
  protected boolean isNear(double volts) {
    return true;
  }

  /** Stator current, useful for detecting a game piece by current spike. */
  public double getCurrentAmps() {
    return inputs.statorCurrentAmps;
  }
}
