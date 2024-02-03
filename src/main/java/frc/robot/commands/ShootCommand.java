package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final Timer time = new Timer();

    public ShootCommand(ShooterSubsystem ss) {
        m_ShooterSubsystem = ss;
        addRequirements(m_ShooterSubsystem);
    }

    

    @Override
    public void initialize() {
        time.restart();
        m_ShooterSubsystem.Shoot();
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("time", time.get());
        if(time.hasElapsed(2))
        {
            time.stop();
            m_ShooterSubsystem.Stop();
            return true;
        }
        return false;
    }
}
