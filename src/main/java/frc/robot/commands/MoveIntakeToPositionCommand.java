package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GyroConstants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;

public class MoveIntakeToPositionCommand extends PIDCommand {

    public MoveIntakeToPositionCommand(IntakeArmSubsystem armSubsystem, double targetPosition) {
        this(armSubsystem, targetPosition, 0.1);
    }

    public MoveIntakeToPositionCommand(IntakeArmSubsystem armSubsystem, double targetPosition, double ff) {
        super(
                new PIDController(ArmConstants.kTurnP, ArmConstants.kTurnI, ArmConstants.kTurnD),
                // Close loop on heading
                armSubsystem::getPosition,
                // Set reference to target
                targetPosition,
                // Pipe output to turn robot
                output -> armSubsystem.moveArm(((1.0 - ff) * output) + (Math.signum(output) * ff)),
                // Require the drive
                armSubsystem);
 
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(0.005);
    }
    
    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }
}