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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.LimelightHelpers;
import frc.utils.SwerveUtils;
import frc.utils.VisionUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro;
  public static double speedAdjustVal = 1;
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private ChassisSpeeds m_prevTarget = new ChassisSpeeds();

  private final Encoder m_ArmEncoder = new 
  Encoder(IntakeArmConstants.kIntakeArmEncoderADioId, IntakeArmConstants.kIntakeArmEncoderBDioId);


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(AHRS gyro) {
    m_gyro = gyro;
    m_odometry= new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        SmartDashboard.putBoolean(  "IMU_Connected",        m_gyro.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    m_gyro.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              m_gyro.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            m_gyro.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             m_gyro.getRoll());
        SmartDashboard.putNumber(   "IMU_Angle",            getAngle());
        SmartDashboard.putBoolean("Has Targed", LimelightHelpers.getTV(""));
        SmartDashboard.putNumber("tX", LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("tY", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("Target Angle To Turn : FR", VisionUtils.calculateAngle(m_gyro));
        SmartDashboard.putNumber("Distance From Target Range", VisionUtils.calculateDistance());
        if((Math.abs(VisionUtils.calculateDistance()) <= 3) && (Math.abs(LimelightHelpers.getTX("")) <= 1))
        {
            SmartDashboard.putBoolean("Ready To Shoot", true);
        }else
        {
          SmartDashboard.putBoolean("Ready To Shoot", false);
        }
        
          SmartDashboard.putBoolean("In Range", (Math.abs(VisionUtils.calculateDistance()) <= 3));
        SmartDashboard.putNumber("speed adjust", speedAdjustVal);
        SmartDashboard.putNumber("Max Speed", getMaxSpeed());
        SmartDashboard.putNumber("Encoder Distance" , m_ArmEncoder.getDistance());
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
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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

    // Convert the commanded speeds into the correct units for the drivetrain
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    // Get the target chassis speeds relative to the robot
    final ChassisSpeeds vel = ( fieldRelative ?
      ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngle()) )
        : new ChassisSpeeds( xSpeed, ySpeed, rot )
    );

    // Rate limit if applicable
    if(rateLimit) {
      final double
        currentTime = WPIUtilJNI.now() * 1e-6,
        elapsedTime = currentTime - m_prevTime;
      SwerveUtils.RateLimitVelocity(
        vel, m_prevTarget, elapsedTime,
        DriveConstants.kMagnitudeSlewRate, DriveConstants.kRotationalSlewRate);
      m_prevTime = currentTime;
      m_prevTarget = vel;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(vel);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

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
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    m_ArmEncoder.reset();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getAngle() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  private double getMaxSpeed()
  {
    return DriveConstants.kMaxSpeedMetersPerSecond * speedAdjustVal;
  }
}
