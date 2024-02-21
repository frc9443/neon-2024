package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final double m_Xspeed;
    private final double m_Yspeed;
    private final double m_Rot;
    private final boolean m_FieldRelative;

    public DriveCommand(DriveSubsystem ds, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        m_DriveSubsystem = ds;
        m_Xspeed = xSpeed;
        m_Yspeed = ySpeed;
        m_Rot = rot;
        m_FieldRelative = fieldRelative;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        m_DriveSubsystem.drive(m_Xspeed, m_Yspeed, m_Rot, m_FieldRelative, false);
    }

}
