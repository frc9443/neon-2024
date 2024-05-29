package frc.robot.commands;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.LimelightHelpers;

public class TurnToAprilTagCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final VisionSubsystem m_VisionSubsystem;

    public TurnToAprilTagCommand(DriveSubsystem ds, VisionSubsystem vs) {
        m_DriveSubsystem = ds;
        m_VisionSubsystem = vs;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.getBoolean("Locked On", m_VisionSubsystem.lockedOn());
        double angleDelta = 0;
        double distanceDelta = 0;
        
        if (m_VisionSubsystem.hasSpeakerTag()) {
            angleDelta = m_VisionSubsystem.getAngleToSpeakerTag();
            distanceDelta = m_VisionSubsystem.getDistanceToShootingPosition();
            double rot = angleDelta * Math.PI / 180 * -.8;
            double xSpeed = 0;
            if (Math.abs(rot) < 0.1) {
                if (distanceDelta > 0) {
                    xSpeed = -Math.max(.05, Math.min(.5, distanceDelta * 0.5));
                } else {
                    xSpeed = -Math.min(-.05, Math.max(-.5, distanceDelta * 0.5));
                }
            }
            SmartDashboard.putNumber("trying xSpeed", xSpeed);
            SmartDashboard.putNumber("tryingRot", rot);
            
            m_DriveSubsystem.drive(
                    xSpeed,
                    0,
                    rot, false, false);
        } else {
            m_DriveSubsystem.drive(0, 0, 0, true, false);
            return;
        }
    }
    

    @Override
    public boolean isFinished() {
        return !m_VisionSubsystem.hasSpeakerTag() || m_VisionSubsystem.lockedOn();
    }
}
