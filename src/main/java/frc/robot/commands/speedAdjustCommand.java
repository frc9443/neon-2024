// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class speedAdjustCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_DriveSubsystem;
  private boolean adjustSpeed;
  public speedAdjustCommand(DriveSubsystem subsystem, boolean adjustUp) {
    m_DriveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
    adjustSpeed = adjustUp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(adjustSpeed = true)
    {
        DriveSubsystem.speedAdjustVal += .2;
        if (DriveSubsystem.speedAdjustVal > 1.21) {
            DriveSubsystem.speedAdjustVal = 1.2;
        }
    }
    else if (adjustSpeed = false)
    {
        DriveSubsystem.speedAdjustVal +=  -.2;
        if (DriveSubsystem.speedAdjustVal < .79) {
            DriveSubsystem.speedAdjustVal = .8;
        }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
