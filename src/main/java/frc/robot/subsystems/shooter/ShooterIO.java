package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    enum ShooterAngle {
        HIGH,
        LOW
    }

    @AutoLog
    class ShooterIOInputs {
        public double velocityTopLeft = 0.0;
        public double appliedVoltsTopLeft = 0.0;
        public double currentAmpsTopLeft = 0.0;
        public double velocityBottomLeft = 0.0;
        public double appliedVoltsBottomLeft = 0.0;
        public double currentAmpsBottomLeft = 0.0;
        public double velocityTopRight = 0.0;
        public double appliedVoltsTopRight = 0.0;
        public double currentAmpsTopRight = 0.0;
        public double velocityBottomRight = 0.0;
        public double appliedVoltsBottomRight = 0.0;
        public double currentAmpsBottomRight = 0.0;

        public ShooterAngle angle = ShooterAngle.HIGH;
    }
    
    default void updateInputs(ShooterIOInputs inputs) {}
    default void shoot(double upperVoltage, double lowerVoltage) {}
    default void stop() {}
    default void setAngle(ShooterAngle position) {}
}
