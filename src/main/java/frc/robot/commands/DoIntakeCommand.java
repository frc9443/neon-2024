package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.intake_arm.IntakeArm;

/**
 * See constructor for details:
 * {@link DoIntakeCommand#DoIntakeCommand(IntakeSubsystem intake, IntakeArm arm)}
 */
public class DoIntakeCommand extends Command {

    IntakeSubsystem intake;
    IntakeArm arm;

    /**
     * This command deploys the intake arm and runs the intake rollers until a note is captured.
     * Once a note is captured, the intake arm is retracted and the command ends.
     * If interrupted, the intake is stopped and the arm will retract to the load position.
     *
     */
    public DoIntakeCommand(IntakeSubsystem intake, IntakeArm arm) {
        this.intake = intake;
        this.arm = arm;
        addRequirements(intake, arm);
    }

    public void initialize() {
        arm.setGoal(IntakeArm.Goal.INTAKE);
    }

    public void execute() {
        if (intake.hasNote()) {
            arm.setGoal(IntakeArm.Goal.LOAD);
            return; // we're done
        }
        if (arm.isOut()) {
            intake.ingest();
        }
    }

    public boolean isFinished() {
        return intake.hasNote();
    }

    public void end(boolean interrupted) {
        intake.stop();
        arm.setGoal(IntakeArm.Goal.LOAD);
    }

}
