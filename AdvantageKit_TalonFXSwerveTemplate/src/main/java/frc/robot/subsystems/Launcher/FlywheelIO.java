package frc.robot.subsystems.Launcher;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    
    @AutoLog
    public static class FlywheelIOInputs {
        double[] statorCurrentAmps = new double[2];
        double[] velocityRPM = new double[2];
    }

    public void setDutyCycle(double doody);

    public default void updateInputs(FlywheelIOInputs inputs) {}
}
