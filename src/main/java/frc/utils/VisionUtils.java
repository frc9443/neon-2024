package frc.utils;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;

public class VisionUtils {
    public static Double lastCalcTarget = null;
    public static double calculateAngle(AHRS gyro)
    {
        LimelightHelpers.setPipelineIndex("", 0);
        if (VisionSubsystem.timerGet() < .001) {
            VisionSubsystem.timerStart();
            double tx = LimelightHelpers.getTX("");  
            double yaw = gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
            double target=(yaw + tx);
            lastCalcTarget = -target;
        }

        if(VisionSubsystem.timerGet() > .1)
            if(LimelightHelpers.getTV(""))
            {
            double tx = LimelightHelpers.getTX("");  
            double yaw = gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
            double target=(yaw + tx);
            lastCalcTarget = -target;
            VisionSubsystem.timerRestart();
            }
        
        return (lastCalcTarget == null ? gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : lastCalcTarget);
    }
    public static double calculateNoteAngle(AHRS gyro)
    {
       LimelightHelpers.setPipelineIndex("", 1);
       if(LimelightHelpers.getTV(""))
        {
          double tx = LimelightHelpers.getTX("");  
          double yaw = gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
          double target=(yaw + tx);
          lastCalcTarget = -target;
        }
        
        return (lastCalcTarget == null ? gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : lastCalcTarget);
    }
    public static double calculateDistance()
    {
        double h1 = VisionConstants.heightOfCamera;
        double h2 = VisionConstants.heightOfCenterSpeaker;
        double a1 = VisionConstants.angleOfCamera;
        double a2 = LimelightHelpers.getTY("");
        double angleInRadians = (a1+ a2) * (3.14159 / 180.0);
        return ((h2-h1) / Math.tan(angleInRadians) - VisionConstants.targetDistanceToShoot);
    }
}