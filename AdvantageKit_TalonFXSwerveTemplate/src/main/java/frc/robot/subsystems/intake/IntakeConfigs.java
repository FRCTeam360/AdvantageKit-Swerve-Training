package frc.robot.subsystems.intake;

import frc.lib.mechanism.MechanismConfig;

/** Intake configs, one factory per robot. RobotContainer picks one. */
public final class IntakeConfigs {
  private IntakeConfigs() {}

  /** The training robot's roller (CAN 3). */
  public static MechanismConfig trainingBot() {
    return MechanismConfig.builder("Intake", 3)
        .rotations()
        .reduction(3.0)
        .brakeMode(false)
        .currentLimits(60.0, 40.0)
        .simRoller(0.001)
        .build();
  }

  /** Same roller on the woodbot; only the CAN ID differs. */
  public static MechanismConfig woodbot2027() {
    return trainingBot().toBuilder()
        .canId(20) // TODO: set once the woodbot is wired
        .build();
  }
}
