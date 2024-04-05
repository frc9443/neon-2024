package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final Timer time = new Timer();

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        time.restart();
        if(m_ShooterSubsystem.getIsShooterUp())
            m_IntakeSubsystem.expel(8);
        else
            m_IntakeSubsystem.expel(8.5);
        m_ShooterSubsystem.Shoot(6.5, 6.5);
    }

    // @Override
    // public void execute(){
    //     if(time.hasElapsed(.2))
    //         m_IntakeSubsystem.expel(8);
    // }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("time", time.get());
        if (time.hasElapsed(.525)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        time.stop();
        m_ShooterSubsystem.Stop();
        m_IntakeSubsystem.stop();
    }
}
