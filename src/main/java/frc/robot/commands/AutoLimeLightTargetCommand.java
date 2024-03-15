package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.LimelightHelpers;

public class AutoLimeLightTargetCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final DriveSubsystem m_DriveSubsystem;
    public AutoLimeLightTargetCommand(DriveSubsystem ds, IntakeSubsystem intake) {
        m_DriveSubsystem = ds;
        m_IntakeSubsystem = intake;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {

        double tx = 0;
        double ty = 0;
        double ta = 0;

        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        // Publish data for viewing on the dashboard


        // Rotate the drivebase to center within +/- 1 degree
        double rot = tx * Math.PI/180 * -.8;
        double ySpeed = -Math.max(.2, 0.02 * Math.min(ty,15));

        SmartDashboard.putNumber("Auto Rotation", rot);

        m_DriveSubsystem.drive(ySpeed, 0, rot, false, false);
    }

      // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.hasNote();
  }
}
