// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  private final static Timer timer = new Timer();
  private double distanceToSpeakerTag;
  private boolean hasSpeakerTag;
  private double angleToSpeaker;

  private PhotonCamera camera;
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public static void timerStart() {
    timer.start();
  }

  public static boolean timerHalfSecondCheck() {
    return timer.hasElapsed(.5);
  }

  public static void timerRestart() {
    timer.restart();
  }

  public static double timerGet() {
    return timer.get();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getDistanceToSpeaker()
  {
    return distanceToSpeakerTag;
  }
  public boolean hasSpeakerTag(){
    return hasSpeakerTag;
  }
  public double getAngleToSpeakerTag(){
    return angleToSpeaker;
  }

}
