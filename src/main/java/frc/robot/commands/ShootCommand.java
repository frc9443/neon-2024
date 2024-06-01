package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Pneumatics pneumatics;
    private final Timer time = new Timer();

    public ShootCommand(Shooter shooter, Intake intake, Pneumatics pneumatics) {
        this.shooter = shooter;
        this.intake = intake;
        this.pneumatics = pneumatics; // read-only, so do not add to requirements
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        time.restart();
        intake.expel( pneumatics.isShooterUp() ? 8 : 8.5 );
        shooter.shoot(6.5, 6.5);
    }

    @Override
    public boolean isFinished() {
        return time.hasElapsed(.525);
    }

    @Override
    public void end(boolean interrupted) {
        time.stop();
        shooter.stop();
        intake.stop();
    }
}
