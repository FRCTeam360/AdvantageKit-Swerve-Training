package frc.lib.mechanism;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mechanism.MechanismConfig.Gains;
import frc.lib.util.LoggedTunableNumber;
import java.util.EnumMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Base class for a subsystem driven by one motor (plus followers). Subclasses are tiny: they pass a
 * config, an IO and their {@code Goal} enum to {@code super}, and add only mechanism-specific
 * helpers.
 *
 * <p><b>Goals.</b> Each subclass declares an enum of named targets (volts for a roller, speed for a
 * flywheel, position for an arm). Every goal value is a {@link LoggedTunableNumber} under {@code
 * /Tuning/<Name>/Goals/<GOAL>}, so it can be adjusted live in AdvantageScope.
 *
 * <p><b>Commands.</b>
 *
 * <ul>
 *   <li>{@link #setGoal} switches to a goal and finishes immediately. The goal stays latched until
 *       something changes it. Use for autos and {@code onTrue}.
 *   <li>{@link #goToAndWait} switches to a goal and finishes when the mechanism gets there. Use in
 *       autos before the next step.
 *   <li>{@link #runGoal} holds a goal only while the command runs. Use for {@code whileTrue}.
 *   <li>{@link #runVoltage} drives open loop while the command runs. Use for first bring-up.
 *   <li>{@link #stop} clears the goal.
 * </ul>
 *
 * Disabling the robot clears the goal, so nothing jumps when the robot is enabled again.
 */
public abstract class Mechanism<G extends Enum<G> & Mechanism.Goal> extends SubsystemBase {
  /** Implemented by each subclass's Goal enum. */
  public interface Goal {
    /** The goal's default value, in the units this mechanism's goals use. */
    double value();
  }

  protected final MechanismConfig config;
  protected final MotorIO io;
  protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  private final Class<G> goalType;
  private final EnumMap<G, LoggedTunableNumber> goalValues;
  private final LoggedTunableNumber kP, kI, kD, kS, kV, kA, kG;

  private final Alert disconnectedAlert;
  private final Alert followerDisconnectedAlert;
  private final Debouncer atGoalDebouncer;

  private G goal = null;
  private Double manualVolts = null;
  private boolean atGoal = false;
  private G atGoalComputedFor = null;

  protected Mechanism(MechanismConfig config, MotorIO io, Class<G> goalType) {
    super(config.name);
    this.config = config;
    this.io = io;
    this.goalType = goalType;

    goalValues = new EnumMap<>(goalType);
    for (G g : goalType.getEnumConstants()) {
      goalValues.put(g, new LoggedTunableNumber(config.name + "/Goals/" + g.name(), g.value()));
    }

    String gainsKey = config.name + "/Gains/";
    Gains gains = config.gains;
    kP = new LoggedTunableNumber(gainsKey + "kP", gains.kP());
    kI = new LoggedTunableNumber(gainsKey + "kI", gains.kI());
    kD = new LoggedTunableNumber(gainsKey + "kD", gains.kD());
    kS = new LoggedTunableNumber(gainsKey + "kS", gains.kS());
    kV = new LoggedTunableNumber(gainsKey + "kV", gains.kV());
    kA = new LoggedTunableNumber(gainsKey + "kA", gains.kA());
    kG = new LoggedTunableNumber(gainsKey + "kG", gains.kG());
    // The IO was configured with these gains already; record them so we don't re-send them.
    LoggedTunableNumber.ifChanged(hashCode(), values -> {}, kP, kI, kD, kS, kV, kA, kG);

    disconnectedAlert = new Alert(config.name + " motor disconnected.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert(config.name + " follower motor disconnected.", AlertType.kError);
    atGoalDebouncer = new Debouncer(config.settleTimeSeconds);
  }

  // ---------------------------------------------------------------------------------------------
  // Hooks for the three mechanism types
  // ---------------------------------------------------------------------------------------------

  /** Sends a goal value to the motor. Called every loop while a goal is set. */
  protected abstract void applySetpoint(double value);

  /** What to do with no goal and no manual voltage. */
  protected abstract void applyIdle();

  /** Whether the mechanism is within tolerance of a goal value right now. */
  protected abstract boolean isNear(double value);

  /** Called whenever the goal is cleared (stop, disable, end of manual control). */
  protected void onGoalCleared() {}

  // ---------------------------------------------------------------------------------------------
  // Loop
  // ---------------------------------------------------------------------------------------------

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    disconnectedAlert.set(config.installed && !inputs.connected);
    boolean followersOk = true;
    for (boolean connected : inputs.followersConnected) followersOk &= connected;
    followerDisconnectedAlert.set(config.installed && !followersOk);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        v -> io.setGains(new Gains(v[0], v[1], v[2], v[3], v[4], v[5], v[6])),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG);

    if (DriverStation.isDisabled()) {
      clearGoalNow();
      io.stop();
    } else if (manualVolts != null) {
      io.setVoltage(manualVolts);
    } else if (goal != null) {
      applySetpoint(goalValue(goal));
    } else {
      applyIdle();
    }

    atGoal = atGoalDebouncer.calculate(goal != null && isNear(goalValue(goal)));
    atGoalComputedFor = goal;

    Logger.recordOutput(getName() + "/Goal", goal == null ? "NONE" : goal.name());
    Logger.recordOutput(getName() + "/GoalValue", goal == null ? 0.0 : goalValue(goal));
    Logger.recordOutput(getName() + "/AtGoal", atGoal());
    Logger.recordOutput(getName() + "/ManualVolts", manualVolts == null ? 0.0 : manualVolts);
  }

  // ---------------------------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------------------------

  /** Switches to a goal and finishes immediately. The goal stays latched. */
  public Command setGoal(G newGoal) {
    return runOnce(() -> setGoalNow(newGoal)).withName(getName() + " " + newGoal);
  }

  /** Switches to a goal and finishes once the mechanism is there. The goal stays latched. */
  public Command goToAndWait(G newGoal) {
    return runOnce(() -> setGoalNow(newGoal))
        .andThen(Commands.waitUntil(() -> atGoal(newGoal)))
        .withName(getName() + " " + newGoal + " and wait");
  }

  /** Holds a goal while the command runs, then clears it. */
  public Command runGoal(G newGoal) {
    return startEnd(() -> setGoalNow(newGoal), this::clearGoalNow)
        .withName(getName() + " run " + newGoal);
  }

  /** Clears the goal. A roller or flywheel stops; a position mechanism holds where it is. */
  public Command stop() {
    return runOnce(this::clearGoalNow).withName(getName() + " stop");
  }

  /** Open loop voltage while the command runs. For bring-up and checking directions. */
  public Command runVoltage(DoubleSupplier volts) {
    return runEnd(() -> manualVolts = volts.getAsDouble(), this::clearGoalNow)
        .withName(getName() + " manual");
  }

  /**
   * Registers every goal as PathPlanner named commands: "{@code <Name> <GOAL>}" latches the goal
   * and "{@code <Name> <GOAL> and wait}" also waits for it. Call before building autos.
   */
  public void registerNamedCommands() {
    for (G g : goalType.getEnumConstants()) {
      NamedCommands.registerCommand(getName() + " " + g.name(), setGoal(g));
      NamedCommands.registerCommand(getName() + " " + g.name() + " and wait", goToAndWait(g));
    }
    NamedCommands.registerCommand(getName() + " stop", stop());
  }

  // ---------------------------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------------------------

  /** The current goal, or null when there is none. */
  public G getGoal() {
    return goal;
  }

  /** Whether the mechanism has settled at its current goal. */
  public boolean atGoal() {
    return atGoal && atGoalComputedFor == goal;
  }

  /** Whether the mechanism has settled at a specific goal. */
  public boolean atGoal(G g) {
    return goal == g && atGoal();
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  /** The live (possibly tuned) value of a goal. */
  public double goalValue(G g) {
    return goalValues.get(g).get();
  }

  protected void setGoalNow(G newGoal) {
    manualVolts = null;
    goal = newGoal;
  }

  protected void clearGoalNow() {
    manualVolts = null;
    goal = null;
    onGoalCleared();
  }

  /** For subclasses that need open loop control, such as homing. Null returns to normal. */
  protected void setManualVolts(Double volts) {
    manualVolts = volts;
  }
}
