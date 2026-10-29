package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Launcher.FlywheelIO.FlywheelIOInputs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX[] motors = 
    {
        new TalonFX(9999999, new CANBus()),
        new TalonFX(999999999, new CANBus())
    };
    private final StatusSignal<AngularVelocity> rightVelocitySignal;
    private final StatusSignal<AngularVelocity> leftVelocitySignal;
    private final StatusSignal<Current> rightStatorCurrentSignal;
    private final StatusSignal<Current> leftStatorCurrentSignal;

    FlywheelIOTalonFX() {
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(defaultConfig);
        }
        rightVelocitySignal = motors[0].getVelocity();
        leftVelocitySignal = motors[1].getVelocity();
        rightStatorCurrentSignal = motors[0].getStatorCurrent();
        leftStatorCurrentSignal = motors[1].getStatorCurrent();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.statorCurrentAmps[1] = leftStatorCurrentSignal.getValueAsDouble();
        inputs.statorCurrentAmps[0] = rightStatorCurrentSignal.getValueAsDouble();
        inputs.velocityRPM[1] = leftVelocitySignal.getValueAsDouble();
        inputs.velocityRPM[0] = rightVelocitySignal.getValueAsDouble();
    }

    @Override
    public void setDutyCycle(double doody) {

    }


}
