package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EjectCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final Intake intake;
    private final Timer time = new Timer();

    public EjectCommand(ShooterSubsystem shooterSubsystem, Intake intakeSubsystem) {
        m_ShooterSubsystem = shooterSubsystem;
        intake = intakeSubsystem;
        addRequirements(m_ShooterSubsystem, intake);
    }

    @Override
    public void initialize() {
        time.restart();
        m_ShooterSubsystem.Shoot(6, 6);
        intake.ingest(6.5);
    }

    @Override
    public boolean isFinished() {
        return time.hasElapsed(.8);
    }

    @Override
    public void end(boolean interrupted) {
        time.stop();
        m_ShooterSubsystem.Stop();
        intake.stop();
    }
}
