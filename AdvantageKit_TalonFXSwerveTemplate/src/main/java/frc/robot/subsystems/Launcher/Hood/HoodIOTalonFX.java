package frc.robot.subsystems.Launcher.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(35);
    hoodMotor.getConfigurator().apply(config);

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    statorCurrentSignal = hoodMotor.getStatorCurrent();
    supplyCurrentSignal = hoodMotor.getSupplyCurrent();
    motorVoltageSignal = hoodMotor.getMotorVoltage();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        motorVoltageSignal);
    inputs.position = Units.rotationsToDegrees(positionSignal.getValueAsDouble());
    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.statorCurrent = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.velocity = Units.rotationsToDegrees(velocitySignal.getValueAsDouble());
    inputs.voltage = motorVoltageSignal.getValueAsDouble();
    inputs.connected =
        BaseStatusSignal.isAllGood(
            positionSignal,
            velocitySignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            motorVoltageSignal);
  }

  @Override
  public void setDutyCycle(double duty) {
    hoodMotor.set(duty);
  }

  @Override
  public void setPosition(double positionDegrees) {
    hoodMotor.setControl(positionVoltage.withPosition(Units.degreesToRotations(positionDegrees)));
  }
}
