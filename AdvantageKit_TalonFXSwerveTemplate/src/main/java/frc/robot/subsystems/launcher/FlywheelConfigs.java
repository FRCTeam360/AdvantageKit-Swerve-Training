package frc.robot.subsystems.launcher;

import frc.lib.mechanism.MechanismConfig;

/** Flywheel configs, one factory per robot. RobotContainer picks one. */
public final class FlywheelConfigs {
  private FlywheelConfigs() {}

  /** The training robot's launcher: two Krakens on one shaft (CAN 32 and 33), direct drive. */
  public static MechanismConfig trainingBot() {
    return MechanismConfig.builder("Flywheel", 32)
        .follower(33, true)
        .rotations()
        .reduction(1.0)
        .brakeMode(false)
        .currentLimits(80.0, 40.0)
        .kV(0.12) // volts per rotation/s; Kraken free speed is about 100 rps at 12 V
        .kP(0.2)
        .tolerance(0.0, 2.0) // within 2 rps counts as at speed
        .settleTime(0.1)
        .simRoller(0.01)
        .build();
  }

  public static MechanismConfig woodbot2027() {
    return trainingBot().toBuilder()
        .canId(30) // TODO: set once the woodbot is wired
        .clearFollowers()
        .follower(31, true)
        .build();
  }
}
