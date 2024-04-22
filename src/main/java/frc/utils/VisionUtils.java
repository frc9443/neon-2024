package frc.utils;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.VisionConstants;

public class VisionUtils {
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