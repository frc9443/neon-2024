package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
   @AutoLog
   class ClimberIOInputs {
        public double velocityLeft = 0.0;
        public double appliedVoltsLeft = 0.0;
        public double currentAmpsLeft = 0.0;
        public double velocityRight = 0.0;
        public double appliedVoltsRight = 0.0;
        public double currentAmpsRight = 0.0;
   } 
   
    default void updateInputs(ClimberIOInputs inputs) {}
    default void climb(double leftRate, double rightRate) {}
    default void stop() {}
}
