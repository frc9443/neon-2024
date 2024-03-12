// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ActivateIntakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_IntakeSubsystem;
  private final BlinkinSubsystem m_BlinkinSubsystem;

  public ActivateIntakeCommand(IntakeSubsystem subsystem, BlinkinSubsystem blinkin) {
    m_IntakeSubsystem = subsystem;
    m_BlinkinSubsystem = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_IntakeSubsystem.ingest(7);
      m_BlinkinSubsystem.setColor(.69);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_IntakeSubsystem.hasNote()){
      m_BlinkinSubsystem.setColor(.77);
      return m_IntakeSubsystem.hasNote();
    }  
    return false;
  }
}
