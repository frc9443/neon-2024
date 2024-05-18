package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double velocityLeft = 0.0;
        public double appliedVoltsLeft = 0.0;
        public double currentAmpsLeft = 0.0;
        public double velocityRight = 0.0;
        public double appliedVoltsRight = 0.0;
        public double currentAmpsRight = 0.0;
        public boolean hasNote = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    default void ingest(double voltage) {}
    default void expel(double voltage) {}
    default void stop() {}
}