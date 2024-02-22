// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualOverrideCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeArmSubsystem m_IntakeArmSubsystem;
  private final ClimberSubsystem m_ClimberSubsystem;
  private final XboxController m_OperatorController;
  private final IntakeSubsystem m_IntakeSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualOverrideCommand(IntakeArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem,
      XboxController controller, IntakeSubsystem intakeSubsystem) {
    m_IntakeArmSubsystem = armSubsystem;
    m_ClimberSubsystem = climberSubsystem;
    m_OperatorController = controller;
    m_IntakeSubsystem = intakeSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakeArmRate = m_OperatorController.getRightY() / 3;
    m_IntakeArmSubsystem.moveArm(intakeArmRate);
    // double intakeRate = m_OperatorController.getRightX() / 2;
    // m_IntakeSubsystem.run(intakeRate);

    double climberRate = m_OperatorController.getLeftY() * 0.75;
    if (Math.abs(climberRate) > 0.05) {
      m_ClimberSubsystem.moveClimber(climberRate);
    } else {
      double leftClimberRate = m_OperatorController.getLeftTriggerAxis() / 5;
      double rightClimberRate = m_OperatorController.getRightTriggerAxis() / 5;
      m_ClimberSubsystem.moveClimber(leftClimberRate, rightClimberRate);
    }
    ;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberSubsystem.stopClimber();
    m_IntakeArmSubsystem.stopArm();
    m_IntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_OperatorController.getLeftBumper() == false) {
      m_ClimberSubsystem.stopClimber();
      m_IntakeArmSubsystem.stopArm();
      m_IntakeSubsystem.stop();

      return true;
    }
    return false;
  }
}
