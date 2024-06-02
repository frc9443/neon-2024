package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean hasSpeakerTag;
        public double distanceToSpeakerTag;
        public double angleToSpeaker;
    }

    default public void updateInputs(VisionIOInputs inputs) {}
}
