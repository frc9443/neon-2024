// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualOverrideCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeArmSubsystem m_IntakeArmSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  private final XboxController m_OperatorController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualOverrideCommand(IntakeArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem, XboxController controller) {
    m_IntakeArmSubsystem = armSubsystem;
    m_ClimberSubsystem = climberSubsystem;
    m_OperatorController = controller;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_OperatorController.getRightX() != 0)
    {
        double rate = m_OperatorController.getRightX() / 2.0;
        m_IntakeArmSubsystem.moveArm(rate);
    }
    if(m_OperatorController.getLeftX() != 0)
    {
        double rate = m_OperatorController.getLeftX() / 10;
        m_ClimberSubsystem.moveClimber(rate);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_OperatorController.getLeftBumper() == false)
    {
        m_ClimberSubsystem.stopClimber();
        m_IntakeArmSubsystem.stopArm();
        return true;
    }
    return false;
  }
}
