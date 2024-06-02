package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.utils.LoggedTunableNumber;
import frc.utils.VirtualSubsystem;

public class Vision extends VirtualSubsystem {

    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    LoggedTunableNumber idealShootingDistance;

    public Vision(VisionIO io) {
        this.io = io;
        idealShootingDistance = new LoggedTunableNumber("Ideal Shooting Distance",
                VisionConstants.idealShootingDistance);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean hasSpeakerTag() {
        return inputs.hasSpeakerTag;
    }

    public double getAngleToSpeaker() {
        return inputs.angleToSpeaker;
    }

    @AutoLogOutput
    public double getDistanceToShootingPosition() {
        return idealShootingDistance.getAsDouble() - inputs.distanceToSpeakerTag;
    }

    @AutoLogOutput
    public boolean lockedOn() {
        if (Math.abs(inputs.angleToSpeaker) > VisionConstants.shootingAngleVariance) {
            return false;
        }
        return (Math.abs(getDistanceToShootingPosition()) < VisionConstants.shootingDistanceVariance);
    }
}
