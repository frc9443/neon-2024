package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class VisionIOPhoton implements VisionIO {

    PhotonCamera camera;

    public VisionIOPhoton() {
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getTargets().stream()
                    .filter(t -> (t.getFiducialId() == 7 || t.getFiducialId() == 4)).findFirst().orElse(null);
            if (target != null) {
                inputs.hasSpeakerTag = true;
                inputs.angleToSpeaker = target.getYaw();
                inputs.distanceToSpeakerTag = PhotonUtils.calculateDistanceToTargetMeters(
                        VisionConstants.heightOfCamera,
                        VisionConstants.heightOfCenterSpeaker,
                        VisionConstants.angleOfCamera,
                        Units.degreesToRadians(target.getPitch()));
            } else {
                inputs.hasSpeakerTag = false;
            }
        }
    }

}
