// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType simMode = RobotType.SIM;
  public static final RobotType currentMode = RobotBase.isReal() ? getRobotType() : RobotType.SIM;

  // === QUESTNAV POSE SEED ===
  // QuestNav is seeded to a fixed corner as a backup; QuestNav's AprilTag support may let it
  // self-locate instead. The seed is offset this far in from the far corner edges. TODO:
  // measure the Quest headset mount on the 2025 drivebase and use its real robot dimensions.
  public static final double QUEST_SEED_EDGE_MARGIN_METERS = 0.7;

  public static final class SerialAddressConstants {
    public static final String DRIVEBASE2025_SERIAL_ADDRESS = "03126d42";
    public static final String DRIVEBASE2024_SERIAL_ADDRESS = "03064DB8";
  }

  private static RobotType getRobotType() {
    if (!Robot.isReal()) {
      robotType = Constants.RobotType.SIM;
      return robotType;
    }

    String serialAddress = HALUtil.getSerialNumber();

    switch (serialAddress) {
      case SerialAddressConstants.DRIVEBASE2025_SERIAL_ADDRESS -> robotType =
          RobotType.COMPBOT_2025;
      case SerialAddressConstants.DRIVEBASE2024_SERIAL_ADDRESS -> robotType =
          RobotType.COMPBOT_2024;
      default -> {
        DriverStation.reportWarning(
            "Unknown serial \"" + serialAddress + "\", defaulting to 2025 compbot", false);
        robotType = RobotType.COMPBOT_2025;
      }
    }
    return robotType;
  }

  static RobotType robotType;

  public static enum RobotType {
    /** Running on a real robot. */
    REAL,

    COMPBOT_2026,

    COMPBOT_2025,

    COMPBOT_2024,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
