package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArmSubsystem;

public class MoveIntakeToPositionCommand extends Command {
    private double m_distance = 10; // Height is in inches
    private double m_speed = .2; // Speed is a _positive_ number, between 0 and 1
    private IntakeArmSubsystem m_IntakeArmSubsystem;
    

    private final double m_tolerance = 1;

    public MoveIntakeToPositionCommand(IntakeArmSubsystem armSubsystem) {
        m_IntakeArmSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(m_IntakeArmSubsystem.getPosition() < m_distance) {
            // The lift is lower than it should be.
            // Raise the lift.
            m_IntakeArmSubsystem.moveArm(m_speed);
        }
        if(m_IntakeArmSubsystem.getPosition() > m_distance) {
            // The lift is higher than it should be.
            // Lower the lift.
            m_IntakeArmSubsystem.moveArm(m_speed);;
        }
    }

    @Override
    public boolean isFinished() {
        final double distanceToHeight = m_IntakeArmSubsystem.getPosition() - m_distance; // This can be negative depending on whether the lift is higher or lower than the desired height.
        final boolean isWithinTolerance = Math.abs(distanceToHeight) < m_tolerance;
        return isWithinTolerance;
    }
}