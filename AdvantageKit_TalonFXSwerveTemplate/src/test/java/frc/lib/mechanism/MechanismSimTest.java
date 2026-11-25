package frc.lib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfigs;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfigs;
import frc.robot.subsystems.launcher.Flywheel;
import frc.robot.subsystems.launcher.FlywheelConfigs;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs the example mechanisms against the TalonFX simulator in real time, the same way the robot
 * sim does, and checks they reach their goals.
 */
class MechanismSimTest {
  private static Arm arm;
  private static Flywheel flywheel;
  private static Intake intake;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    var armConfig = ArmConfigs.woodbot2027();
    var flywheelConfig = FlywheelConfigs.woodbot2027();
    var intakeConfig = IntakeConfigs.woodbot2027();
    arm = new Arm(armConfig, new MotorIOTalonFXSim(armConfig));
    flywheel = new Flywheel(flywheelConfig, new MotorIOTalonFXSim(flywheelConfig));
    intake = new Intake(intakeConfig, new MotorIOTalonFXSim(intakeConfig));
    runFor(1.0); // let the simulated CAN devices come up
  }

  /** Steps the robot loop at 50 Hz until the condition holds or the time runs out. */
  private static boolean runUntil(BooleanSupplier done, double timeoutSeconds) {
    long end = System.nanoTime() + (long) (timeoutSeconds * 1e9);
    while (System.nanoTime() < end) {
      Unmanaged.feedEnable(100);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      CommandScheduler.getInstance().run();
      if (done.getAsBoolean()) return true;
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        return false;
      }
    }
    return false;
  }

  private static void runFor(double seconds) {
    runUntil(() -> false, seconds);
  }

  private static boolean run(Command command, double timeoutSeconds) {
    CommandScheduler.getInstance().schedule(command);
    return runUntil(() -> !command.isScheduled(), timeoutSeconds);
  }

  @Test
  void armReachesGoalsAndHomes() {
    assertTrue(run(arm.goToAndWait(Arm.Goal.SCORE), 4.0), "arm never reached SCORE");
    assertEquals(80.0, arm.getPosition(), 2.0);

    // Holds the goal after the command ends (no default command, nothing drops)
    runFor(0.5);
    assertEquals(80.0, arm.getPosition(), 2.0);

    assertTrue(run(arm.goToAndWait(Arm.Goal.STOW), 4.0), "arm never reached STOW");
    assertEquals(-15.0, arm.getPosition(), 2.0);

    assertTrue(run(arm.home(), 4.0), "homing never finished");
    assertTrue(arm.isHomed());
    assertEquals(-20.0, arm.getPosition(), 1.0);
  }

  @Test
  void flywheelReachesSpeed() {
    var shoot = flywheel.runGoal(Flywheel.Goal.SHOOT);
    CommandScheduler.getInstance().schedule(shoot);
    assertTrue(runUntil(flywheel::atSpeed, 4.0), "flywheel never got to speed");
    assertEquals(50.0, flywheel.getVelocity(), 2.0);
    shoot.cancel();
  }

  @Test
  void intakeSpinsWhileHeld() {
    var intaking = intake.runGoal(Intake.Goal.INTAKE);
    CommandScheduler.getInstance().schedule(intaking);
    runFor(0.5);
    assertTrue(intake.getVelocity() > 1.0, "intake didn't spin: " + intake.getVelocity());
    intaking.cancel();
    runFor(1.0);
    assertTrue(intake.getGoal() == null);
  }
}
