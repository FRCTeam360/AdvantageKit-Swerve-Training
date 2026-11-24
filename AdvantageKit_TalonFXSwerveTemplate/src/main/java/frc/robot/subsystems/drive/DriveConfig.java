package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.Drivebase2025;
import frc.robot.generated.TunerConstants;

/**
 * Everything the drive code needs to know about one physical drivebase.
 *
 * <p>Nothing in the drive package reads drivetrain constants statically. RobotContainer picks one
 * DriveConfig for the robot it is running on and passes it to the Drive subsystem. The IO
 * implementations do not take a DriveConfig; RobotContainer hands each one only the pieces it needs
 * (for example one module's constants and the CAN bus).
 *
 * <p>To add a drivebase, generate its file with Tuner X, rename the class, add a factory method
 * below, and add a case to the robot type switch in RobotContainer.
 *
 * @param canBus CAN bus the drive motors, encoders and gyro are on
 * @param drivetrainConstants Tuner X drivetrain constants (Pigeon 2 ID and configuration)
 * @param speedAt12Volts Robot speed at 12 V applied output
 * @param frontLeft Tuner X constants for the front left module
 * @param frontRight Tuner X constants for the front right module
 * @param backLeft Tuner X constants for the back left module
 * @param backRight Tuner X constants for the back right module
 * @param robotMassKg Robot mass, used by PathPlanner and the maple-sim chassis simulation
 * @param robotMoi Robot moment of inertia in kg*m^2, used by PathPlanner
 * @param wheelCof Wheel coefficient of friction, used by PathPlanner and maple-sim
 */
public record DriveConfig(
    CANBus canBus,
    SwerveDrivetrainConstants drivetrainConstants,
    LinearVelocity speedAt12Volts,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        frontLeft,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        frontRight,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        backLeft,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        backRight,
    double robotMassKg,
    double robotMoi,
    double wheelCof) {

  /** The 2025 drivebase, from its Tuner X generated file. */
  public static DriveConfig compbot2025() {
    return new DriveConfig(
        Drivebase2025.kCANBus,
        Drivebase2025.DrivetrainConstants,
        Drivebase2025.kSpeedAt12Volts,
        Drivebase2025.FrontLeft,
        Drivebase2025.FrontRight,
        Drivebase2025.BackLeft,
        Drivebase2025.BackRight,
        74.088,
        6.883,
        1.2);
  }

  /**
   * The AdvantageKit template's example values. These do not describe a real robot; they are a
   * placeholder for drivebases that have no Tuner X file of their own yet.
   */
  public static DriveConfig templateExample() {
    return new DriveConfig(
        TunerConstants.kCANBus,
        TunerConstants.DrivetrainConstants,
        TunerConstants.kSpeedAt12Volts,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight,
        74.088,
        6.883,
        1.2);
  }

  /** Odometry sample rate in Hz for this drivebase's CAN bus. */
  public double odometryFrequency() {
    return PhoenixOdometryThread.frequencyFor(canBus);
  }

  /** Module positions relative to the robot center, in FL, FR, BL, BR order. */
  public Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(frontLeft.LocationX, frontLeft.LocationY),
      new Translation2d(frontRight.LocationX, frontRight.LocationY),
      new Translation2d(backLeft.LocationX, backLeft.LocationY),
      new Translation2d(backRight.LocationX, backRight.LocationY)
    };
  }

  /** Distance in meters from the robot center to the farthest module. */
  public double driveBaseRadius() {
    double radius = 0.0;
    for (Translation2d translation : moduleTranslations()) {
      radius = Math.max(radius, translation.getNorm());
    }
    return radius;
  }
}
