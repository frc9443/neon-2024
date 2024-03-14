package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.LimelightHelpers;

public class FollowLimeLightTargetCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final XboxController m_DriverController;

    public FollowLimeLightTargetCommand(DriveSubsystem ds, XboxController DCont) {
        m_DriveSubsystem = ds;
        m_DriverController = DCont;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {

        double tx = 0;
        double ty = 0;
        double ta = 0;

        // Check for targets
        boolean hasTargets = LimelightHelpers.getTV("");
        SmartDashboard.putBoolean("Target acquired", hasTargets);

        // Gather target data
        if (hasTargets) {
            tx = LimelightHelpers.getTX("");
            ty = LimelightHelpers.getTY("");
            ta = LimelightHelpers.getTA("");
        } else {
            tx = 0;
            ty = 0;
            ta = 0;
        }

        // Publish data for viewing on the dashboard
        SmartDashboard.putNumber("Target yaw", tx);
        SmartDashboard.putNumber("Target pitch", ty);
        SmartDashboard.putNumber("Target Distance", ta);

        // Rotate the drivebase to center within +/- 1 degree
        double rot = tx * Math.PI/180 * -.8;
        //double ySpeed = -Math.max(.2, 0.02 * Math.min(ty,15));
        // double ySpeed = ;
        SmartDashboard.putNumber("Desired Rotation", rot);

        m_DriveSubsystem.drive(
            m_DriverController.getLeftX(),
            m_DriverController.getLeftY(),
            rot, true, false);
    }

}
