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
 * Defines two independent things:
 *
 * <ul>
 *   <li>{@link Mode}: how the code is running (real hardware, physics sim, or log replay). This
 *       decides which IO implementations are created.
 *   <li>{@link RobotType}: which physical robot the code is running on (or pretending to be). This
 *       decides which configuration RobotContainer builds, e.g. which DriveConfig.
 * </ul>
 *
 * On a roboRIO the mode is always REAL and the robot type is detected from the roboRIO serial
 * number. Off the robot, change "simMode" to switch between SIM and REPLAY, and "simRobot" to
 * choose which robot is simulated or replayed.
 */
public final class Constants {
  /** Mode to use when not running on a roboRIO. */
  public static final Mode simMode = Mode.SIM;

  /**
   * Robot to simulate when not running on a roboRIO. For REPLAY, this must match the robot that
   * recorded the log.
   */
  public static final RobotType simRobot = RobotType.COMPBOT_2025;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final RobotType robotType = RobotBase.isReal() ? detectRobotType() : simRobot;

  public static final class SerialAddressConstants {
    public static final String DRIVEBASE2025_SERIAL_ADDRESS = "03126d42";
    public static final String DRIVEBASE2024_SERIAL_ADDRESS = "03064DB8";
  }

  /** Identifies the physical robot from the roboRIO serial number. */
  private static RobotType detectRobotType() {
    String serialAddress = HALUtil.getSerialNumber();

    switch (serialAddress) {
      case SerialAddressConstants.DRIVEBASE2025_SERIAL_ADDRESS:
        return RobotType.COMPBOT_2025;
      case SerialAddressConstants.DRIVEBASE2024_SERIAL_ADDRESS:
        return RobotType.COMPBOT_2024;
      default:
        DriverStation.reportWarning(
            "Unknown serial \"" + serialAddress + "\", defaulting to 2025 compbot", false);
        return RobotType.COMPBOT_2025;
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    COMPBOT_2026,
    COMPBOT_2025,
    COMPBOT_2024
  }
}
