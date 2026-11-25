// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MotorIO;
import frc.lib.mechanism.MotorIOTalonFX;
import frc.lib.mechanism.MotorIOTalonFXSim;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfigs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMapleSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfigs;
import frc.robot.subsystems.launcher.Flywheel;
import frc.robot.subsystems.launcher.FlywheelConfigs;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private final Intake intake;
  private final Arm arm;
  private final Flywheel flywheel;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulation
  private static final Pose2d SIM_START_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);
  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Robot type: the only place that decides which robot's configuration is used. Everything
    // below receives its configuration through its constructor.
    final DriveConfig driveConfig;
    final MechanismConfig intakeConfig;
    final MechanismConfig armConfig;
    final MechanismConfig flywheelConfig;
    switch (Constants.robotType) {
      case WOODBOT_2027:
        // TODO: generate the woodbot's Tuner X file
        driveConfig = DriveConfig.templateExample();
        intakeConfig = IntakeConfigs.woodbot2027();
        armConfig = ArmConfigs.woodbot2027();
        flywheelConfig = FlywheelConfigs.woodbot2027();
        break;

      case COMPBOT_2025:
        driveConfig = DriveConfig.compbot2025();
        intakeConfig = IntakeConfigs.trainingBot();
        armConfig = ArmConfigs.woodbot2027().asNotInstalled();
        flywheelConfig = FlywheelConfigs.trainingBot();
        break;

      case COMPBOT_2024:
      case COMPBOT_2026:
      default:
        // TODO: These robots have no Tuner X file of their own yet
        DriverStation.reportWarning(
            "No drive config for " + Constants.robotType + ", using the template example values",
            false);
        driveConfig = DriveConfig.templateExample();
        intakeConfig = IntakeConfigs.trainingBot();
        armConfig = ArmConfigs.woodbot2027().asNotInstalled();
        flywheelConfig = FlywheelConfigs.trainingBot();
        break;
    }

    // Mode: decides which IO implementations are created
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                driveConfig,
                new GyroIOPigeon2(driveConfig.drivetrainConstants(), driveConfig.canBus()),
                new ModuleIOTalonFX(driveConfig.frontLeft(), driveConfig.canBus()),
                new ModuleIOTalonFX(driveConfig.frontRight(), driveConfig.canBus()),
                new ModuleIOTalonFX(driveConfig.backLeft(), driveConfig.canBus()),
                new ModuleIOTalonFX(driveConfig.backRight(), driveConfig.canBus()),
                (robotPose) -> {});

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(driveConfig.drivetrainConstants(), driveConfig.canBus()),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft, driveConfig.canBus()),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight, driveConfig.canBus()),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft, driveConfig.canBus()),
        // new ModuleIOTalonFXS(TunerConstants.BackRight, driveConfig.canBus()));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // maple-sim simulates the chassis, so the modules and gyro read from that simulation
        driveSimulation =
            new SwerveDriveSimulation(Drive.getMapleSimConfig(driveConfig), SIM_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                driveConfig,
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOMapleSim(driveConfig.frontLeft(), driveSimulation.getModules()[0]),
                new ModuleIOMapleSim(driveConfig.frontRight(), driveSimulation.getModules()[1]),
                new ModuleIOMapleSim(driveConfig.backLeft(), driveSimulation.getModules()[2]),
                new ModuleIOMapleSim(driveConfig.backRight(), driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                driveConfig,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Mechanisms: the IO for each comes from its config and the mode (see motorIO below)
    intake = new Intake(intakeConfig, motorIO(intakeConfig));
    arm = new Arm(armConfig, motorIO(armConfig));
    flywheel = new Flywheel(flywheelConfig, motorIO(flywheelConfig));

    // PathPlanner named commands, e.g. "Arm SCORE and wait". Must come before building autos.
    intake.registerNamedCommands();
    arm.registerNamedCommands();
    flywheel.registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Picks the IO for a mechanism: real TalonFXs on the robot, simulated TalonFXs in sim, and an IO
   * that does nothing for log replay or for a mechanism this robot doesn't have.
   */
  private static MotorIO motorIO(MechanismConfig config) {
    if (!config.installed) return new MotorIO() {};
    switch (Constants.currentMode) {
      case REAL:
        return new MotorIOTalonFX(config);
      case SIM:
        return new MotorIOTalonFXSim(config);
      default:
        return new MotorIO() {};
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    // Mechanisms
    controller.rightTrigger().whileTrue(intake.runGoal(Intake.Goal.INTAKE));
    controller.leftTrigger().whileTrue(intake.runGoal(Intake.Goal.EJECT));
    controller.leftBumper().whileTrue(flywheel.runGoal(Flywheel.Goal.SHOOT));
    controller.povDown().onTrue(arm.setGoal(Arm.Goal.STOW));
    controller.povLeft().onTrue(arm.setGoal(Arm.Goal.INTAKE));
    controller.povUp().onTrue(arm.setGoal(Arm.Goal.SCORE));
    controller.start().onTrue(arm.home());
    // Bring-up: hold Y and use the right stick to drive the arm open loop (max 4 V)
    controller.y().whileTrue(arm.runVoltage(() -> -controller.getRightY() * 4.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  /** Moves the simulated robot back to its starting pose and resets the field. Sim only. */
  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(SIM_START_POSE);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /** Steps the physics simulation and logs the true robot pose and game pieces. Sim only. */
  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
