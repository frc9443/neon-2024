package frc.robot.subsystems.intake_arm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    class IntakeArmIOInputs {
        public double position = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(IntakeArmIOInputs inputs) {}
    default void moveArm(double voltage) {}
    default void stopArm() {}
}