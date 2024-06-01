package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {

    enum ShooterAngle {
        HIGH,
        LOW
    }

    @AutoLog
    class PneumaticsIOInputs {
        public boolean isEnabled = false;
        public boolean isFull = false;
        public double pressure = 0.0;
        public double minPressure = 0.0;
        public double maxPressure = 0.0;
        public double appliedCurrent = 0.0;
        public double appliedVolts = 0.0;
        public ShooterAngle shooterAngle = ShooterAngle.HIGH;
    }

    default void updateInputs(PneumaticsIOInputs inputs) {
    }

    default void setPressure(double min, double max) {
    }

    default void setShooterAngle(ShooterAngle position) {
    }

    default void stop() {
    }
}
