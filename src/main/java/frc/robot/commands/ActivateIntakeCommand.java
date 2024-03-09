// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ActivateIntakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_IntakeSubsystem;
  private double speed;
  private boolean hasNote = false;

  public ActivateIntakeCommand(IntakeSubsystem subsystem, double setSpeed) {
    m_IntakeSubsystem = subsystem;
    speed = setSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_IntakeSubsystem.run(-7);
      hasNote = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_IntakeSubsystem.rightLSGet() == true) || (m_IntakeSubsystem.leftLSGet() == true)){
      hasNote = true;
    }
    SmartDashboard.putBoolean("Has Note", hasNote);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_IntakeSubsystem.rightLSGet() == false) || (m_IntakeSubsystem.leftLSGet() == false)){
      m_IntakeSubsystem.stop();
      return true;
    }
    return false;
  }
}
