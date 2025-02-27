package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EjectCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final Timer time = new Timer();

    public EjectCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        time.restart();
        m_ShooterSubsystem.Shoot(6, 6);
        m_IntakeSubsystem.ingest(6.5);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("time", time.get());
        if (time.hasElapsed(.8)) {
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
