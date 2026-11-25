package frc.lib.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be changed live from AdvantageScope or Elastic under {@code /Tuning/<key>}
 * while {@link Constants#tuningMode} is on. With tuning mode off it always returns its default, so
 * a stray dashboard edit can't change the robot at an event.
 *
 * <p>The dashboard value is an AdvantageKit logged input, so log replay sees the same values the
 * robot saw.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String TABLE = "/Tuning";

  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastValues = new HashMap<>();

  public LoggedTunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    this.dashboardNumber =
        Constants.tuningMode ? new LoggedNetworkNumber(TABLE + "/" + key, defaultValue) : null;
  }

  public double get() {
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Whether the value changed since the last call with this id. Each caller should pass its own id
   * (for example {@code hashCode()}) so callers don't consume each other's changes.
   */
  public boolean hasChanged(int id) {
    double value = get();
    Double last = lastValues.put(id, value);
    return last == null || last != value;
  }

  /** Runs the action with every current value if any of them changed. */
  public static void ifChanged(int id, Consumer<double[]> action, LoggedTunableNumber... numbers) {
    boolean changed = false;
    for (var number : numbers) {
      // Evaluate every number so each records its latest value.
      changed |= number.hasChanged(id);
    }
    if (changed) {
      double[] values = new double[numbers.length];
      for (int i = 0; i < numbers.length; i++) values[i] = numbers[i].get();
      action.accept(values);
    }
  }
}
