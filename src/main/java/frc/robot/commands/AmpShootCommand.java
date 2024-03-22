// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AmpShootCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IntakeSubsystem m_IntakeSubsystem;
    private final IntakeArmSubsystem m_IntakeArmSubsystem;
    private final Timer time = new Timer();
    

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AmpShootCommand(IntakeSubsystem intakeSubsystem, IntakeArmSubsystem armSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        m_IntakeArmSubsystem = armSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time.restart();
        m_IntakeSubsystem.expel(9.5); // TODO: tune voltage
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
        SmartDashboard.putNumber("time", time.get());
        if (time.hasElapsed(.5)) {
            return true;
        }
        return false;
    }
}
