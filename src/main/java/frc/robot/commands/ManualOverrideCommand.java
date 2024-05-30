// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake_arm.IntakeArm;

/** An example command that uses an example subsystem. */
public class ManualOverrideCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeArm intakeArm;
  private final Climber climber;
  private final XboxController m_OperatorController;
  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeArm        The IntakeArm subsystem used by this command.
   * @param climberSubsystem The Climber subsystem used by this command.
   * @param controller       The XBox controller used by this command.
   * @param intakeSubsystem  Intake subsystem used by this command.
   */
  public ManualOverrideCommand(IntakeArm intakeArm, Climber climber,
      XboxController controller, Intake intakeSubsystem) {
    this.intakeArm = intakeArm;
    this.climber = climber;
    m_OperatorController = controller;
    intake = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakeArmRate = m_OperatorController.getRightY() / 3;
    intakeArm.acceptTeleopInput(intakeArmRate);
    // double intakeRate = m_OperatorController.getRightX() / 2;
    // intake.run(intakeRate);

    double climberRate = m_OperatorController.getLeftY() * 0.75;
    if (Math.abs(climberRate) > 0.05) {
      climber.climb(climberRate);
    } else {
      double leftClimberRate = m_OperatorController.getLeftTriggerAxis() / 3.5;
      double rightClimberRate = m_OperatorController.getRightTriggerAxis() / 3.5;
      climber.climb(leftClimberRate, rightClimberRate);
    }
    ;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    intakeArm.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_OperatorController.getLeftBumper() == false) {
      climber.stop();
      intakeArm.stop();
      intake.stop();

      return true;
    }
    return false;
  }
}
