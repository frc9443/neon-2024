package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinSubsystem;

public class ChangeColorCommand extends Command {
    private final BlinkinSubsystem m_BlinkinSubsystem;
    private boolean isFinished = false;
    private double value;

    public ChangeColorCommand(BlinkinSubsystem BS, double value)
    {
        m_BlinkinSubsystem = BS; 
        this.value = value;
    }

    @Override
    public void initialize() {
      isFinished = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_BlinkinSubsystem.set(value);;
      isFinished = true;
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isFinished;
    }
    
}
