package frc.lib.mechanism;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulated {@link MotorIOTalonFX}. The real TalonFX code runs unchanged, including its onboard
 * closed loop; only the physical mechanism is replaced by a WPILib physics model chosen by the
 * config's {@link MechanismConfig.SimType}. Gains tuned here carry over to the robot.
 *
 * <p>The physics runs at 200 Hz on its own thread so the TalonFX's closed loop sees smooth motion.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX {
  private static final double SIM_PERIOD_SECONDS = 0.005;

  private final TalonFXSimState simState;
  private final double startRotations;
  private final PhysicsModel model;
  private final Notifier simNotifier;
  private double lastTime;

  /** Shared shape of the three WPILib physics models, in mechanism rotations. */
  private interface PhysicsModel {
    void step(double volts, double dt);

    double positionRotations();

    double velocityRps();
  }

  public MotorIOTalonFXSim(MechanismConfig config) {
    super(config);
    simState = leader.getSimState();
    simState.Orientation =
        config.inverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    startRotations = config.startingPosition / config.unitsPerRotation;
    model = createModel(config, startRotations);

    lastTime = Timer.getFPGATimestamp();
    simNotifier = new Notifier(this::updateSim);
    simNotifier.setName(config.name + " sim");
    simNotifier.startPeriodic(SIM_PERIOD_SECONDS);
  }

  private static PhysicsModel createModel(MechanismConfig c, double startRotations) {
    double twoPi = 2.0 * Math.PI;
    // Hard stops: the soft limits if set, otherwise effectively unlimited.
    double minRot = Double.isFinite(c.minPosition) ? c.minPosition / c.unitsPerRotation : -1e6;
    double maxRot = Double.isFinite(c.maxPosition) ? c.maxPosition / c.unitsPerRotation : 1e6;

    switch (c.simType) {
      case ARM:
        {
          var sim =
              new SingleJointedArmSim(
                  c.simMotor,
                  c.reduction,
                  c.simMoiKgMetersSq,
                  c.simArmLengthMeters,
                  minRot * twoPi,
                  maxRot * twoPi,
                  true,
                  startRotations * twoPi);
          return new PhysicsModel() {
            public void step(double volts, double dt) {
              sim.setInputVoltage(volts);
              sim.update(dt);
            }

            public double positionRotations() {
              return sim.getAngleRads() / twoPi;
            }

            public double velocityRps() {
              return sim.getVelocityRadPerSec() / twoPi;
            }
          };
        }
      case ELEVATOR:
        {
          // In meters, unitsPerRotation is the drum circumference.
          double drumRadius = c.unitsPerRotation / twoPi;
          var sim =
              new ElevatorSim(
                  c.simMotor,
                  c.reduction,
                  c.simCarriageMassKg,
                  drumRadius,
                  Math.max(minRot, 0.0) * c.unitsPerRotation,
                  maxRot * c.unitsPerRotation,
                  true,
                  startRotations * c.unitsPerRotation);
          return new PhysicsModel() {
            public void step(double volts, double dt) {
              sim.setInputVoltage(volts);
              sim.update(dt);
            }

            public double positionRotations() {
              return sim.getPositionMeters() / c.unitsPerRotation;
            }

            public double velocityRps() {
              return sim.getVelocityMetersPerSecond() / c.unitsPerRotation;
            }
          };
        }
      case ROLLER:
      default:
        {
          var sim =
              new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(c.simMotor, c.simMoiKgMetersSq, c.reduction),
                  c.simMotor);
          sim.setState(startRotations * twoPi, 0.0);
          return new PhysicsModel() {
            public void step(double volts, double dt) {
              sim.setInputVoltage(volts);
              sim.update(dt);
            }

            public double positionRotations() {
              return sim.getAngularPositionRotations();
            }

            public double velocityRps() {
              return sim.getAngularVelocityRadPerSec() / twoPi;
            }
          };
        }
    }
  }

  private void updateSim() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;
    if (dt <= 0.0 || dt > 0.1) return;

    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    model.step(simState.getMotorVoltage(), dt);

    // The TalonFX already counts startingPosition (set in the constructor), so the raw rotor
    // reading only carries the motion since boot.
    double rotorRotations = (model.positionRotations() - startRotations) * config.reduction;
    simState.setRawRotorPosition(rotorRotations);
    simState.setRotorVelocity(model.velocityRps() * config.reduction);
  }
}
