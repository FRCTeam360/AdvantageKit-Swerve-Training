package frc.lib.mechanism;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/**
 * A mechanism that moves to positions: arms, pivots, wrists, elevators. Goal values are positions
 * in mechanism units. With no goal it holds wherever it is, so it never drops when a command ends.
 *
 * <p>If the config has {@link MechanismConfig.Builder#homing homing}, {@link #home()} drives gently
 * into the hard stop until it stalls, then sets the encoder. That makes a woodbot without absolute
 * encoders safe to boot in any position.
 */
public abstract class PositionMechanism<G extends Enum<G> & Mechanism.Goal> extends Mechanism<G> {
  private static final double HOMING_TIMEOUT_SECONDS = 3.0;
  private static final double HOMING_STALL_SECONDS = 0.25;

  private double holdPosition;
  private boolean homed;

  protected PositionMechanism(MechanismConfig config, MotorIO io, Class<G> goalType) {
    super(config, io, goalType);
    holdPosition = config.startingPosition;
    // Without homing we trust the starting position; with homing, the robot should home first.
    homed = config.homingVolts == 0.0;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/HoldPosition", holdPosition);
    Logger.recordOutput(getName() + "/Homed", homed);
  }

  @Override
  protected void applySetpoint(double position) {
    io.setPosition(position);
  }

  @Override
  protected void applyIdle() {
    io.setPosition(holdPosition);
  }

  @Override
  protected boolean isNear(double position) {
    return Math.abs(inputs.position - position) <= config.positionTolerance;
  }

  @Override
  protected void onGoalCleared() {
    holdPosition = inputs.position;
  }

  public boolean isHomed() {
    return homed;
  }

  /**
   * Drives into the hard stop at the configured homing voltage until the mechanism stops moving,
   * then sets the encoder to the hard stop position. Soft limits are off while homing. Gives up
   * after 3 seconds without changing the encoder.
   */
  public Command home() {
    if (config.homingVolts == 0.0) {
      return Commands.print(getName() + " has no homing configured").ignoringDisable(true);
    }
    var stalled = new Debouncer(HOMING_STALL_SECONDS);
    boolean[] succeeded = {false};
    return startRun(
            () -> {
              succeeded[0] = false;
              stalled.calculate(false);
              setGoalNow(null);
              io.setSoftLimitsEnabled(false);
              setManualVolts(config.homingVolts);
            },
            () -> setManualVolts(config.homingVolts))
        .until(
            () ->
                succeeded[0] =
                    stalled.calculate(Math.abs(inputs.velocity) <= config.velocityTolerance))
        .withTimeout(HOMING_TIMEOUT_SECONDS)
        .finallyDo(
            () -> {
              setManualVolts(null);
              if (succeeded[0]) {
                io.resetPosition(config.homePosition);
                homed = true;
                holdPosition = config.homePosition;
              } else {
                holdPosition = inputs.position;
              }
              io.setSoftLimitsEnabled(true);
            })
        .withName(getName() + " home");
  }

  /** Registers the goals and "{@code <Name> home}" as PathPlanner named commands. */
  @Override
  public void registerNamedCommands() {
    super.registerNamedCommands();
    com.pathplanner.lib.auto.NamedCommands.registerCommand(getName() + " home", home());
  }
}
