package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.LimelightHelpers;

public class FollowAprilTagCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;

    public FollowAprilTagCommand(DriveSubsystem ds) {
        m_DriveSubsystem = ds;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {

        double targetingSpeed = 0.4;
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
        double rot = 0;

        if (tx > 20) {
        } else if (tx < -20) {
        } else if (tx > 10) {
        } else if (tx < -10) {
        } else if (tx > 5) {
        } else if (tx < -5) {
        } else if (ta > .05 & ta < 10) {
        } else if (ta < 20 & ta > 30) {
        }

        if (tx > 1) {
            rot = -.2;
        } else if (tx < -1) {
            rot = 0.2;
        }

        m_DriveSubsystem.drive(0, 0, rot, false, false);
    }

}
