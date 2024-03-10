package frc.utils;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;

public class OffsetGyro {
    private AHRS m_gyro;
    private double m_offset;

    public OffsetGyro(AHRS gyro, double offset) {
        m_gyro = gyro;
        m_offset = offset;
    }

    public OffsetGyro(AHRS gyro) {
        this(gyro, 180.0);
    }

    public void setOffset(double offset) {
        m_offset = wrapAngle(offset);
    }

    private static double wrapAngle(double angle) {
        return (180 + angle) % 360 - 180;
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getAngle() {
        return wrapAngle(m_gyro.getAngle() + m_offset) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    public void reset(){
        m_gyro.reset();
        m_offset = 0.0;
    }
}
