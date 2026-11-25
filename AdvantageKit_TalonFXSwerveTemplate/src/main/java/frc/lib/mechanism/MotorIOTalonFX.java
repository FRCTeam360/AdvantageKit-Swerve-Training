package frc.lib.mechanism;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.mechanism.MechanismConfig.Gains;
import java.util.function.Supplier;

/**
 * {@link MotorIO} for a TalonFX (Kraken or Falcon) leader with any number of TalonFX followers.
 *
 * <p>The TalonFX works in mechanism rotations (SensorToMechanismRatio = reduction). This class
 * converts to and from the config's mechanism units, including the gains, so nothing above the IO
 * layer ever sees motor rotations.
 */
public class MotorIOTalonFX implements MotorIO {
  protected final MechanismConfig config;
  protected final TalonFX leader;
  protected final TalonFX[] followers;
  private final TalonFXConfiguration talonConfig;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<?>[] followerSignals;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public MotorIOTalonFX(MechanismConfig config) {
    this.config = config;
    leader = new TalonFX(config.canId, config.canBus);
    talonConfig = buildTalonConfig(config);
    tryUntilOk(() -> leader.getConfigurator().apply(talonConfig, 0.25));
    tryUntilOk(() -> leader.setPosition(toRotations(config.startingPosition), 0.25));

    followers = new TalonFX[config.followers.size()];
    followerSignals = new StatusSignal<?>[followers.length];
    for (int i = 0; i < followers.length; i++) {
      var f = config.followers.get(i);
      TalonFX follower = new TalonFX(f.canId(), config.canBus);
      tryUntilOk(() -> follower.getConfigurator().apply(talonConfig, 0.25));
      follower.setControl(
          new Follower(
              config.canId,
              f.opposeLeader() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
      followers[i] = follower;
      followerSignals[i] = follower.getStatorCurrent();
    }

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerSignals);
    ParentDevice.optimizeBusUtilizationForAll(leader);
    ParentDevice.optimizeBusUtilizationForAll(followers);
  }

  private static TalonFXConfiguration buildTalonConfig(MechanismConfig config) {
    var c = new TalonFXConfiguration();
    c.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    c.MotorOutput.NeutralMode = config.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    c.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps;
    c.CurrentLimits.StatorCurrentLimitEnable = true;
    c.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimitAmps;
    c.CurrentLimits.SupplyCurrentLimitEnable = true;

    c.Feedback.SensorToMechanismRatio = config.reduction;
    c.Slot0 = slot0(config, config.gains);

    c.MotionMagic.MotionMagicCruiseVelocity = config.cruiseVelocity / config.unitsPerRotation;
    c.MotionMagic.MotionMagicAcceleration = config.acceleration / config.unitsPerRotation;

    c.SoftwareLimitSwitch = softLimits(config, true);
    return c;
  }

  /** Converts gains in mechanism units to the TalonFX's per-rotation gains. */
  private static Slot0Configs slot0(MechanismConfig config, Gains gains) {
    double upr = config.unitsPerRotation;
    var slot = new Slot0Configs();
    slot.kP = gains.kP() * upr;
    slot.kI = gains.kI() * upr;
    slot.kD = gains.kD() * upr;
    slot.kS = gains.kS();
    slot.kV = gains.kV() * upr;
    slot.kA = gains.kA() * upr;
    slot.kG = gains.kG();
    slot.GravityType =
        config.gravityType == MechanismConfig.GravityType.ARM
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;
    if (config.gravityType == MechanismConfig.GravityType.NONE) slot.kG = 0.0;
    return slot;
  }

  private static SoftwareLimitSwitchConfigs softLimits(MechanismConfig config, boolean enabled) {
    var limits = new SoftwareLimitSwitchConfigs();
    if (Double.isFinite(config.maxPosition)) {
      limits.ForwardSoftLimitThreshold = config.maxPosition / config.unitsPerRotation;
      limits.ForwardSoftLimitEnable = enabled;
    }
    if (Double.isFinite(config.minPosition)) {
      limits.ReverseSoftLimitThreshold = config.minPosition / config.unitsPerRotation;
      limits.ReverseSoftLimitEnable = enabled;
    }
    return limits;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    inputs.connected = connectedDebouncer.calculate(status.isOK());

    if (inputs.followersConnected.length != followers.length) {
      inputs.followersConnected = new boolean[followers.length];
    }
    for (int i = 0; i < followers.length; i++) {
      inputs.followersConnected[i] = BaseStatusSignal.refreshAll(followerSignals[i]).isOK();
    }

    inputs.position = position.getValueAsDouble() * config.unitsPerRotation;
    inputs.velocity = velocity.getValueAsDouble() * config.unitsPerRotation;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double position) {
    double target = MathUtil.clamp(position, config.minPosition, config.maxPosition);
    if (config.cruiseVelocity > 0.0) {
      leader.setControl(motionMagicRequest.withPosition(toRotations(target)));
    } else {
      leader.setControl(positionRequest.withPosition(toRotations(target)));
    }
  }

  @Override
  public void setVelocity(double velocity) {
    leader.setControl(velocityRequest.withVelocity(toRotations(velocity)));
  }

  @Override
  public void stop() {
    leader.setControl(neutralRequest);
  }

  @Override
  public void resetPosition(double position) {
    leader.setPosition(toRotations(position));
  }

  @Override
  public void setGains(Gains gains) {
    leader.getConfigurator().apply(slot0(config, gains));
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    if (config.hasSoftLimits()) {
      leader.getConfigurator().apply(softLimits(config, enabled));
    }
  }

  protected double toRotations(double units) {
    return units / config.unitsPerRotation;
  }

  private static void tryUntilOk(Supplier<StatusCode> command) {
    for (int i = 0; i < 5; i++) {
      if (command.get().isOK()) return;
    }
  }
}
