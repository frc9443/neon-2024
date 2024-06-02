// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.DriveConstants;
import frc.utils.LimelightHelpers;
import frc.utils.OffsetGyro;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private MAXSwerveModule m_frontLeft;
    private MAXSwerveModule m_frontRight;
    private MAXSwerveModule m_rearLeft;
    private MAXSwerveModule m_rearRight;

    // The gyro sensor
    private final OffsetGyro m_gyro;
    public static double speedAdjustVal = 1;
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private ChassisSpeeds m_prevTarget = new ChassisSpeeds();

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry;

    public static MAXSwerveModule fromConfig(Class<?> config, String drivingId, String turningId, double angularOffset) {
        try {
            return new MAXSwerveModule(
                    (int) config.getField(drivingId).get(config),
                    (int) config.getField(turningId).get(config),
                    angularOffset);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public DriveSubsystem(OffsetGyro gyro) {
        m_gyro = gyro;

        Class<?> robotDrive = DriveConstants.Neon.class;

        m_frontLeft = fromConfig(robotDrive,
                "kFrontLeftDrivingCanId", "kFrontLeftTurningCanId",
                DriveConstants.kFrontLeftChassisAngularOffset);

        m_frontRight = fromConfig(robotDrive,
                "kFrontRightDrivingCanId", "kFrontRightTurningCanId",
                DriveConstants.kFrontRightChassisAngularOffset);

        m_rearLeft = fromConfig(robotDrive,
                "kRearLeftDrivingCanId", "kRearLeftTurningCanId",
                DriveConstants.kBackLeftChassisAngularOffset);

        m_rearRight = fromConfig(robotDrive,
                "kRearRightDrivingCanId", "kRearRightTurningCanId",
                DriveConstants.kBackRightChassisAngularOffset);

        m_odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.71, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        // SmartDashboard.putBoolean("IMU_Connected", m_gyro.isConnected());
        // SmartDashboard.putBoolean("IMU_IsCalibrating", m_gyro.isCalibrating());
        // SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
        // SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
        // SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());
        // SmartDashboard.putNumber("IMU_Angle", getAngle());

        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void resetPose(Pose2d pose) {
        resetOdometry(pose);
    }

    // Path Builder Required.
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    // Path Builder Drive Command
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit,
                      double maxSpeedMetersPerSecond) {

        // Convert the commanded speeds into the correct units for the drivetrain
        xSpeed *= maxSpeedMetersPerSecond;
        ySpeed *= maxSpeedMetersPerSecond;
        rot *= DriveConstants.kMaxAngularSpeed;

        // Get the target chassis speeds relative to the robot
        final ChassisSpeeds vel = (fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

        // Rate limit if applicable
        if (rateLimit) {
            final double currentTime = WPIUtilJNI.now() * 1e-6,
                    elapsedTime = currentTime - m_prevTime;
            SwerveUtils.RateLimitVelocity(
                    vel, m_prevTarget, elapsedTime,
                    DriveConstants.kMagnitudeSlewRate, DriveConstants.kRotationalSlewRate);
            m_prevTime = currentTime;
            m_prevTarget = vel;
        }
        driveRobotRelative(vel);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

}
