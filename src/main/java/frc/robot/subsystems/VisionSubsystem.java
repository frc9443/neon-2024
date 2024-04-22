// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  private double distanceToSpeakerTag;
  private boolean hasSpeakerTag;
  private double angleToSpeaker;
  private double idealShootingDistance; // Tuning with input from dashboard
  private PhotonCamera camera;

  public VisionSubsystem() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    // Tuning with input from dashboard
    SmartDashboard.setDefaultNumber("Ideal Shooting Distance", 2.8);
  }

  @Override
  public void periodic() {
    // Tuning with input from dashboard 
    idealShootingDistance = SmartDashboard.getNumber("Ideal Shooting Distance", 0);

    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getTargets().stream().filter(t -> (t.getFiducialId() == 7 || t.getFiducialId() == 4)).findFirst().orElse(null);
      if (target != null) {
      hasSpeakerTag = true;
      angleToSpeaker = target.getYaw();
      distanceToSpeakerTag = 
        PhotonUtils.calculateDistanceToTargetMeters(
        Constants.VisionConstants.heightOfCamera,
        Constants.VisionConstants.heightOfCenterSpeaker,
        Constants.VisionConstants.angleOfCamera,
        Units.degreesToRadians(target.getPitch()));
      SmartDashboard.putNumber("Distance to Apriltag", distanceToSpeakerTag);
      SmartDashboard.putNumber("Angle to Apriltag", angleToSpeaker);
      }
      else{
        hasSpeakerTag = false;
      }
    }
  }

  public double getDistanceToSpeaker()
  {
    return distanceToSpeakerTag;
  }
  public double getDistanceToShootingPosition(){
    // return Constants.VisionConstants.idealShootingDistance - distanceToSpeakerTag;
    return idealShootingDistance - distanceToSpeakerTag; // Tuning with input from dashboard
  }
  public boolean hasSpeakerTag(){
    return hasSpeakerTag;
  }
  public double getAngleToSpeakerTag(){
    return angleToSpeaker;
  }
  public boolean lockedOn(){
    if (Math.abs(angleToSpeaker) > Constants.VisionConstants.shootingAngleVariance){
      return false;
    }
    return (Math.abs(getDistanceToShootingPosition()) < Constants.VisionConstants.shootingDistanceVariance);
  }
}
