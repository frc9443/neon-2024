package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class TurnToAprilTagCommand extends Command {
    private final Drive m_DriveSubsystem;
    private final Vision vision;

    public TurnToAprilTagCommand(Drive ds, Vision vs) {
        m_DriveSubsystem = ds;
        vision = vs;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.getBoolean("Locked On", vision.lockedOn());
        double angleDelta = 0;
        double distanceDelta = 0;

        if (vision.hasSpeakerTag()) {
            angleDelta = vision.getAngleToSpeaker();
            distanceDelta = vision.getDistanceToShootingPosition();
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

            m_DriveSubsystem.drive(xSpeed, 0, rot, false);
        } else {
            m_DriveSubsystem.drive(0, 0, 0, true);
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return !vision.hasSpeakerTag() || vision.lockedOn();
    }
}
