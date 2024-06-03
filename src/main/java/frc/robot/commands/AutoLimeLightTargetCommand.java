package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.utils.LimelightHelpers;

public class AutoLimeLightTargetCommand extends Command {
  private final Intake intake;
  private final Drive drive;

  public AutoLimeLightTargetCommand(Drive drive, Intake intake) {
    this.drive = drive;
    this.intake = intake;
    addRequirements(drive);
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
    double rot = tx * Math.PI / 180 * -.8;
    double ySpeed = -Math.max(.3, 0.02 * Math.min(ty, 15));

    SmartDashboard.putNumber("Auto Rotation", rot);

    drive.drive(ySpeed, 0, rot, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
