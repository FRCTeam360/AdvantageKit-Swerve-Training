package frc.lib.mechanism;

/**
 * A mechanism run at a closed loop speed: shooters, flywheels. Goal values are velocities in
 * mechanism units per second (rotations per second with {@code rotations()}). A goal of 0 coasts or
 * brakes instead of actively holding 0.
 *
 * <p>Set kV (and kS) in the config before kP: on a flywheel the feedforward does most of the work.
 */
public abstract class FlywheelMechanism<G extends Enum<G> & Mechanism.Goal> extends Mechanism<G> {
  protected FlywheelMechanism(MechanismConfig config, MotorIO io, Class<G> goalType) {
    super(config, io, goalType);
  }

  @Override
  protected void applySetpoint(double velocity) {
    if (velocity == 0.0) {
      io.stop();
    } else {
      io.setVelocity(velocity);
    }
  }

  @Override
  protected void applyIdle() {
    io.stop();
  }

  @Override
  protected boolean isNear(double velocity) {
    return Math.abs(inputs.velocity - velocity) <= config.velocityTolerance;
  }

  /** Whether the flywheel has settled at its goal speed. Same as {@link #atGoal()}. */
  public boolean atSpeed() {
    return atGoal();
  }
}
