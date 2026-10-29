package frc.robot.subsystems.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    
    Flywheel(FlywheelIO io){
        this.io = io;
    }

    private void setDutyCycle(double duty) {
        io.setDutyCycle(duty);
    }

    public Command setDutyCycleCommand(DoubleSupplier valueSup) {
        return this.runEnd(() -> setDutyCycle(valueSup.getAsDouble()), () -> setDutyCycle(0.0));
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);

    }
}
