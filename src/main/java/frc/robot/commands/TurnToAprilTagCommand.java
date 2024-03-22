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
    private final XboxController m_DriverController;
    private final VisionSubsystem m_VisionSubsystem;

    public TurnToAprilTagCommand(DriveSubsystem ds, VisionSubsystem vs, XboxController DCont) {
        m_DriveSubsystem = ds;
        m_DriverController = DCont;
        m_VisionSubsystem = vs;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {

        double tx = 0;
        double ty = 0;

        
        if (m_VisionSubsystem.hasSpeakerTag()) {
             
                tx = m_VisionSubsystem.getAngleToSpeakerTag();
                ty = m_VisionSubsystem.getDistanceToSpeaker();
            } else {
                tx = 0;
                ty = 0;
            }
        


        // Rotate the drivebase to center within +/- 1 degree
        double rot = tx * Math.PI/180 * -.8;
        double xSpeed = 0;
        // if(ty > 3.1)
        // {
        //     xSpeed = -1;
        // }
        // else{
        //     xSpeed = 0;
        // }
        //double ySpeed = -Math.max(0, 0.05 * Math.min(ty,10));

        m_DriveSubsystem.drive(
            0,
            0,
            rot, true, false);
    }

}
