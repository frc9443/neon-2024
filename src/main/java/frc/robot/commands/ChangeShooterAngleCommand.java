// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.pneumatics.PneumaticsIO.ShooterAngle;

/** An example command that uses an example subsystem. */
public class ChangeShooterAngleCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Pneumatics pneumatics;
  private boolean solonoidOpen = false;
  private boolean isFinished = false;

  public ChangeShooterAngleCommand(Pneumatics pneumatics, boolean isUp) {
    this.pneumatics = pneumatics;
    solonoidOpen = isUp;
    addRequirements(pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (solonoidOpen) {
      pneumatics.setShooterAngle(ShooterAngle.HIGH);
    } else {
      pneumatics.setShooterAngle(ShooterAngle.LOW);
    }
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
