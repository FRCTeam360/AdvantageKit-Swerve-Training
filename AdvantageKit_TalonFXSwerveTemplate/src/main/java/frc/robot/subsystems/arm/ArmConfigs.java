package frc.robot.subsystems.arm;

import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MechanismConfig.GravityType;

/** Arm configs, one factory per robot. RobotContainer picks one. */
public final class ArmConfigs {
  private ArmConfigs() {}

  /**
   * Example woodbot arm: one Kraken through 60:1, resting on a hard stop 20 degrees below
   * horizontal. The gains were tuned in sim and are a starting point for the real arm.
   */
  public static MechanismConfig woodbot2027() {
    return MechanismConfig.builder("Arm", 40) // TODO: set once the woodbot is wired
        .degrees()
        .reduction(60.0)
        .brakeMode(true)
        .currentLimits(60.0, 40.0)
        .kP(0.3) // volts per degree of error
        .kD(0.005)
        .kV(0.02) // volts per degree/s
        .kG(0.25) // volts to hold the arm horizontal
        .gravity(GravityType.ARM)
        .motionMagic(300.0, 600.0) // deg/s, deg/s^2
        .softLimits(-20.0, 110.0)
        .tolerance(2.0, 5.0) // degrees, degrees/s
        .homing(-1.5, -20.0) // drive down at 1.5 V into the hard stop at -20 degrees
        .simArm(0.3, 0.5) // kg*m^2, meters
        .build();
  }
}
