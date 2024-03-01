package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveOverrideCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final double m_Xspeed;
    private final double m_Yspeed;
    private final double m_Rot;
    private final boolean m_FieldRelative;
    private final double m_MaxSpeed;

    public DriveOverrideCommand(DriveSubsystem ds, double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxSpeed) {
        m_DriveSubsystem = ds;
        m_Xspeed = xSpeed;
        m_Yspeed = ySpeed;
        m_Rot = rot;
        m_FieldRelative = fieldRelative;
        m_MaxSpeed = maxSpeed;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        m_DriveSubsystem.DriveOverride(m_Xspeed, m_Yspeed, m_Rot, m_FieldRelative, false, m_MaxSpeed);
    }

}
