package frc.robot.commands;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.LimelightHelpers;

public class TurnToAprilTagCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final XboxController m_DriverController;
    private PhotonCamera m_Camera;

    public TurnToAprilTagCommand(DriveSubsystem ds, XboxController DCont) {
        m_DriveSubsystem = ds;
        m_DriverController = DCont;

        m_Camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {

        double tx = 0;

        var result = m_Camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getTargets().stream().filter(t -> (t.getFiducialId() == 7 || t.getFiducialId() == 4)).findFirst().orElse(null);
            if (target != null) {
                tx = target.getYaw();
            } else {
                tx = 0;
            }
        }

        // Publish data for viewing on the dashboard
        SmartDashboard.putNumber("PICam Target yaw", tx);

        // Rotate the drivebase to center within +/- 1 degree
        double rot = tx * Math.PI/180 * -.8;
        //double ySpeed = -Math.max(.2, 0.02 * Math.min(ty,15));
        // double ySpeed = ;

        m_DriveSubsystem.drive(
            m_DriverController.getLeftX(),
            m_DriverController.getLeftY(),
            rot, true, false);
    }

}
