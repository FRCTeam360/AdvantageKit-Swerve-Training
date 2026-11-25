package frc.lib.mechanism;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.ArrayList;
import java.util.List;

/**
 * Everything the mechanism library needs to know about one motor-driven mechanism: which motors,
 * how they are geared, how they are controlled and how to simulate them.
 *
 * <p><b>Units.</b> Every position, velocity, limit, tolerance and gain is in <i>mechanism
 * units</i>, chosen with {@link Builder#rotations()}, {@link Builder#degrees()} or {@link
 * Builder#meters(double)}. A gain of kP = 0.2 on an arm in degrees means 0.2 volts per degree of
 * error. You never need to think about motor rotations; the IO layer converts.
 *
 * <p>Build one with {@link #builder(String, int)}. To describe a second robot that differs only a
 * little (the woodbot vs the comp bot), start from the first with {@link #toBuilder()} and change
 * only what is different.
 */
public final class MechanismConfig {
  /** Which physics model the simulator uses. */
  public enum SimType {
    /** A spinning shaft with no gravity: intakes, indexers, flywheels. */
    ROLLER,
    /** A pivoting arm with gravity. Position 0 must be horizontal. */
    ARM,
    /** A carriage lifted by a drum or sprocket. Units must be meters. */
    ELEVATOR
  }

  /** Which gravity feedforward kG applies. */
  public enum GravityType {
    NONE,
    /** Constant kG, for elevators. */
    ELEVATOR,
    /** kG * cos(angle), for arms. Position 0 must be horizontal. */
    ARM
  }

  /** A second motor that mirrors the leader. */
  public record Follower(int canId, boolean opposeLeader) {}

  /** Closed loop gains, in mechanism units. */
  public record Gains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public final String name;
  public final boolean installed;

  // Hardware
  public final int canId;
  public final CANBus canBus;
  public final List<Follower> followers;
  public final boolean inverted;
  public final boolean brakeMode;
  public final double statorCurrentLimitAmps;
  public final double supplyCurrentLimitAmps;

  // Gearing and units
  /** Motor rotations per mechanism rotation, e.g. 25.0 for a 25:1 reduction. */
  public final double reduction;
  /** Mechanism units per mechanism rotation (1 for rotations, 360 for degrees). */
  public final double unitsPerRotation;

  public final String unitName;

  // Control
  public final Gains gains;
  public final GravityType gravityType;
  /** Motion Magic cruise velocity in units/s. 0 means use plain position PID instead. */
  public final double cruiseVelocity;
  /** Motion Magic acceleration in units/s^2. */
  public final double acceleration;

  public final double minPosition;
  public final double maxPosition;
  public final double positionTolerance;
  public final double velocityTolerance;
  /** How long the mechanism must stay within tolerance before it counts as at its goal. */
  public final double settleTimeSeconds;

  // Startup and homing
  /** Where the mechanism is assumed to be when the robot boots. */
  public final double startingPosition;
  /** Voltage used to drive into the hard stop when homing. 0 means the mechanism can't home. */
  public final double homingVolts;
  /** Position of the hard stop that homing drives into. */
  public final double homePosition;

  // Simulation
  public final SimType simType;
  public final DCMotor simMotor;
  /** Moment of inertia in kg*m^2 (ROLLER and ARM). */
  public final double simMoiKgMetersSq;
  /** Arm length in meters (ARM). */
  public final double simArmLengthMeters;
  /** Carriage mass in kg (ELEVATOR). */
  public final double simCarriageMassKg;

  private MechanismConfig(Builder b) {
    if (b.reduction <= 0) throw new IllegalArgumentException(b.name + ": reduction must be > 0");
    if (b.simType == SimType.ELEVATOR && !"m".equals(b.unitName)) {
      throw new IllegalArgumentException(b.name + ": elevator sim needs meters(drumDiameter)");
    }
    name = b.name;
    installed = b.installed;
    canId = b.canId;
    canBus = b.canBus;
    followers = List.copyOf(b.followers);
    inverted = b.inverted;
    brakeMode = b.brakeMode;
    statorCurrentLimitAmps = b.statorCurrentLimitAmps;
    supplyCurrentLimitAmps = b.supplyCurrentLimitAmps;
    reduction = b.reduction;
    unitsPerRotation = b.unitsPerRotation;
    unitName = b.unitName;
    gains = new Gains(b.kP, b.kI, b.kD, b.kS, b.kV, b.kA, b.kG);
    gravityType = b.gravityType;
    cruiseVelocity = b.cruiseVelocity;
    acceleration = b.acceleration;
    minPosition = b.minPosition;
    maxPosition = b.maxPosition;
    positionTolerance = b.positionTolerance;
    velocityTolerance = b.velocityTolerance;
    settleTimeSeconds = b.settleTimeSeconds;
    startingPosition = b.startingPosition;
    homingVolts = b.homingVolts;
    homePosition = b.homePosition;
    simType = b.simType;
    simMotor = b.simMotor != null ? b.simMotor : DCMotor.getKrakenX60(1 + b.followers.size());
    simMoiKgMetersSq = b.simMoiKgMetersSq;
    simArmLengthMeters = b.simArmLengthMeters;
    simCarriageMassKg = b.simCarriageMassKg;
  }

  public boolean hasSoftLimits() {
    return Double.isFinite(minPosition) || Double.isFinite(maxPosition);
  }

  /** Starts a config for a mechanism whose leader motor has the given CAN ID. */
  public static Builder builder(String name, int canId) {
    Builder b = new Builder();
    b.name = name;
    b.canId = canId;
    return b;
  }

  /** A copy of this config for a robot that doesn't have this mechanism. */
  public MechanismConfig asNotInstalled() {
    return toBuilder().notInstalled().build();
  }

  /** A builder with every value copied from this config, for making a per-robot variant. */
  public Builder toBuilder() {
    Builder b = new Builder();
    b.name = name;
    b.installed = installed;
    b.canId = canId;
    b.canBus = canBus;
    b.followers.addAll(followers);
    b.inverted = inverted;
    b.brakeMode = brakeMode;
    b.statorCurrentLimitAmps = statorCurrentLimitAmps;
    b.supplyCurrentLimitAmps = supplyCurrentLimitAmps;
    b.reduction = reduction;
    b.unitsPerRotation = unitsPerRotation;
    b.unitName = unitName;
    b.kP = gains.kP();
    b.kI = gains.kI();
    b.kD = gains.kD();
    b.kS = gains.kS();
    b.kV = gains.kV();
    b.kA = gains.kA();
    b.kG = gains.kG();
    b.gravityType = gravityType;
    b.cruiseVelocity = cruiseVelocity;
    b.acceleration = acceleration;
    b.minPosition = minPosition;
    b.maxPosition = maxPosition;
    b.positionTolerance = positionTolerance;
    b.velocityTolerance = velocityTolerance;
    b.settleTimeSeconds = settleTimeSeconds;
    b.startingPosition = startingPosition;
    b.homingVolts = homingVolts;
    b.homePosition = homePosition;
    b.simType = simType;
    b.simMotor = simMotor;
    b.simMoiKgMetersSq = simMoiKgMetersSq;
    b.simArmLengthMeters = simArmLengthMeters;
    b.simCarriageMassKg = simCarriageMassKg;
    return b;
  }

  /** Fluent builder. Every setter returns the builder so calls can be chained. */
  public static final class Builder {
    private String name;
    private boolean installed = true;
    private int canId;
    private CANBus canBus = new CANBus();
    private final List<Follower> followers = new ArrayList<>();
    private boolean inverted = false;
    private boolean brakeMode = true;
    private double statorCurrentLimitAmps = 80.0;
    private double supplyCurrentLimitAmps = 40.0;
    private double reduction = 1.0;
    private double unitsPerRotation = 1.0;
    private String unitName = "rot";
    private double kP, kI, kD, kS, kV, kA, kG;
    private GravityType gravityType = GravityType.NONE;
    private double cruiseVelocity = 0.0;
    private double acceleration = 0.0;
    private double minPosition = Double.NEGATIVE_INFINITY;
    private double maxPosition = Double.POSITIVE_INFINITY;
    private double positionTolerance = 0.0;
    private double velocityTolerance = 0.0;
    private double settleTimeSeconds = 0.06;
    private double startingPosition = 0.0;
    private double homingVolts = 0.0;
    private double homePosition = 0.0;
    private SimType simType = SimType.ROLLER;
    private DCMotor simMotor = null;
    private double simMoiKgMetersSq = 0.004;
    private double simArmLengthMeters = 0.5;
    private double simCarriageMassKg = 5.0;

    private Builder() {}

    // ---- Hardware ----

    public Builder name(String name) {
      this.name = name;
      return this;
    }

    /** Marks the mechanism as not on this robot. It gets an IO that does nothing. */
    public Builder notInstalled() {
      this.installed = false;
      return this;
    }

    public Builder canId(int canId) {
      this.canId = canId;
      return this;
    }

    public Builder canBus(CANBus canBus) {
      this.canBus = canBus;
      return this;
    }

    /** Adds a follower motor. Set opposeLeader when it faces the other way on the shaft. */
    public Builder follower(int canId, boolean opposeLeader) {
      followers.add(new Follower(canId, opposeLeader));
      return this;
    }

    public Builder clearFollowers() {
      followers.clear();
      return this;
    }

    /** Flips which way is positive. Positive should be up / out / intaking. */
    public Builder inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    public Builder brakeMode(boolean brakeMode) {
      this.brakeMode = brakeMode;
      return this;
    }

    public Builder currentLimits(double statorAmps, double supplyAmps) {
      this.statorCurrentLimitAmps = statorAmps;
      this.supplyCurrentLimitAmps = supplyAmps;
      return this;
    }

    // ---- Gearing and units ----

    /** Motor rotations per mechanism rotation. A 5:1 then 3:1 gearbox is 15.0. */
    public Builder reduction(double reduction) {
      this.reduction = reduction;
      return this;
    }

    /** Positions in mechanism rotations, velocities in rotations per second. */
    public Builder rotations() {
      unitsPerRotation = 1.0;
      unitName = "rot";
      return this;
    }

    /** Positions in degrees, velocities in degrees per second. */
    public Builder degrees() {
      unitsPerRotation = 360.0;
      unitName = "deg";
      return this;
    }

    /**
     * Positions in meters of carriage travel. Pass the pitch diameter of the drum or sprocket the
     * reduction ends at. For a multi-stage cascade, multiply the diameter by the number of stages.
     */
    public Builder meters(double drumDiameterMeters) {
      unitsPerRotation = Math.PI * drumDiameterMeters;
      unitName = "m";
      return this;
    }

    // ---- Control ----

    /** Proportional gain: volts per unit of error. */
    public Builder kP(double kP) {
      this.kP = kP;
      return this;
    }

    public Builder kI(double kI) {
      this.kI = kI;
      return this;
    }

    /** Derivative gain: volts per unit/s of error. */
    public Builder kD(double kD) {
      this.kD = kD;
      return this;
    }

    /** Static friction feedforward: volts. */
    public Builder kS(double kS) {
      this.kS = kS;
      return this;
    }

    /** Velocity feedforward: volts per unit/s. */
    public Builder kV(double kV) {
      this.kV = kV;
      return this;
    }

    /** Acceleration feedforward: volts per unit/s^2. */
    public Builder kA(double kA) {
      this.kA = kA;
      return this;
    }

    /** Gravity feedforward in volts. Use with {@link #gravity(GravityType)}. */
    public Builder kG(double kG) {
      this.kG = kG;
      return this;
    }

    public Builder gravity(GravityType gravityType) {
      this.gravityType = gravityType;
      return this;
    }

    /** Use Motion Magic (smooth trapezoid moves) with these limits, in units/s and units/s^2. */
    public Builder motionMagic(double cruiseVelocity, double acceleration) {
      this.cruiseVelocity = cruiseVelocity;
      this.acceleration = acceleration;
      return this;
    }

    /** Soft limits. The motor controller refuses to drive past them. */
    public Builder softLimits(double minPosition, double maxPosition) {
      this.minPosition = minPosition;
      this.maxPosition = maxPosition;
      return this;
    }

    /** How close counts as "at goal", in units and units/s. */
    public Builder tolerance(double position, double velocity) {
      this.positionTolerance = position;
      this.velocityTolerance = velocity;
      return this;
    }

    public Builder settleTime(double seconds) {
      this.settleTimeSeconds = seconds;
      return this;
    }

    // ---- Startup and homing ----

    /** Where the mechanism is assumed to be when the robot boots. */
    public Builder startingPosition(double position) {
      this.startingPosition = position;
      return this;
    }

    /**
     * Lets the mechanism home by driving gently into a hard stop until it stalls, then setting its
     * position. Also sets the starting position to the hard stop.
     */
    public Builder homing(double volts, double hardStopPosition) {
      this.homingVolts = volts;
      this.homePosition = hardStopPosition;
      this.startingPosition = hardStopPosition;
      return this;
    }

    // ---- Simulation ----

    /** Overrides the simulated motors. Defaults to one Kraken X60 per motor. */
    public Builder simMotor(DCMotor motor) {
      this.simMotor = motor;
      return this;
    }

    public Builder simRoller(double moiKgMetersSq) {
      simType = SimType.ROLLER;
      simMoiKgMetersSq = moiKgMetersSq;
      return this;
    }

    public Builder simArm(double moiKgMetersSq, double armLengthMeters) {
      simType = SimType.ARM;
      simMoiKgMetersSq = moiKgMetersSq;
      simArmLengthMeters = armLengthMeters;
      return this;
    }

    public Builder simElevator(double carriageMassKg) {
      simType = SimType.ELEVATOR;
      simCarriageMassKg = carriageMassKg;
      return this;
    }

    public MechanismConfig build() {
      return new MechanismConfig(this);
    }
  }
}
