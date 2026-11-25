# Mechanism library

Write a config and a short Goal enum, and you get a working, logged, simulated, tunable subsystem.
It's based on the 254, 1678 and 6328 public code (see the bottom of this file).

| You have... | Extend | Goal values are |
|---|---|---|
| Intake, indexer, conveyor | `RollerMechanism` | volts |
| Shooter flywheel | `FlywheelMechanism` | units/s (rps with `rotations()`) |
| Arm, pivot, wrist, elevator | `PositionMechanism` | position (deg, m, rot) |

## Adding a mechanism (about 15 minutes)

Copy `subsystems/arm/` (for a position mechanism) or `subsystems/intake/` (for a roller) and rename.

**1. The class.** Declare the named goals. That's the whole class:

```java
public class Wrist extends PositionMechanism<Wrist.Goal> {
  public enum Goal implements frc.lib.mechanism.Mechanism.Goal {
    STOW(90.0), SCORE(-30.0);
    private final double degrees;
    Goal(double degrees) { this.degrees = degrees; }
    public double value() { return degrees; }
  }
  public Wrist(MechanismConfig config, MotorIO io) { super(config, io, Goal.class); }
}
```

**2. The config.** Add one factory method per robot in `WristConfigs`:

```java
public static MechanismConfig woodbot2027() {
  return MechanismConfig.builder("Wrist", 41)
      .degrees().reduction(45.0)
      .kP(0.2).kV(0.015).kG(0.2).gravity(GravityType.ARM)
      .motionMagic(400, 800).softLimits(-40, 100).tolerance(2, 5)
      .homing(1.5, 100.0)           // optional: home up into the hard stop at 100 deg
      .simArm(0.05, 0.3)
      .build();
}
```

For the comp bot, start from the woodbot and change only what differs:
`woodbot2027().toBuilder().canId(51).reduction(50.0).build()`.

**3. RobotContainer.** Pick the config in the robot type switch, then build it:

```java
wrist = new Wrist(wristConfig, motorIO(wristConfig));
wrist.registerNamedCommands();
```

Run the sim. The wrist shows up in AdvantageScope under `Wrist/`, and its goals and gains are under `/Tuning/Wrist/`.

## Commands you get for free

| Command | Use it for |
|---|---|
| `setGoal(G)` | `onTrue` buttons, autos. Latches the goal. |
| `goToAndWait(G)` | Autos: move and wait until settled before the next step. |
| `runGoal(G)` | `whileTrue` buttons. Holds the goal, then stops. |
| `runVoltage(supplier)` | First power-on: check direction with a joystick. |
| `stop()` | Clear the goal. Position mechanisms hold where they are. |
| `home()` | Position mechanisms with `homing(...)` configured. |

PathPlanner named commands are registered as `Arm SCORE`, `Arm SCORE and wait`, `Arm stop` and `Arm home`.

## Bring-up checklist for a new mechanism on the woodbot

1. Set the CAN ID in its config. Confirm `<Name>/connected` is true in AdvantageScope.
2. Hold the `runVoltage` button at low voltage. Positive must be **up / out / intaking**. If it isn't, set `.inverted(true)`.
3. Move the mechanism by hand. `<Name>/position` must read the right units. If it doesn't, fix `reduction` (motor turns per mechanism turn).
4. Set soft limits a little inside the hard stops.
5. Tune live in `/Tuning/<Name>/Gains/`, in this order: kS, then kG (arm and elevator), then kV, then kP, then kD. Copy the final values back into the config, because tunables reset on reboot.
6. Tune goal values live in `/Tuning/<Name>/Goals/`, then copy those back too.

## Units

Every number in the config is in mechanism units: degrees, meters, or rotations. Gains are too, so kP = 0.3 on an arm in degrees means 0.3 V per degree of error. The IO layer converts to motor rotations. You never use motor rotations yourself.

`ARM` gravity and the arm sim need position 0 to be horizontal.

## Where things came from

- **254:** a single `MotorIO` for every mechanism; the sim IO extends the real TalonFX IO, so the onboard closed loop runs in sim; configs hold all the numbers.
- **1678:** a base class per mechanism type (servo, flywheel, roller), named setpoints, and stall-current homing.
- **6328:** AdvantageKit structure, goal enums backed by `LoggedTunableNumber`, a devbot (woodbot) as its own robot type, and an empty IO for replay or missing hardware.
