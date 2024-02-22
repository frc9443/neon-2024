// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.LimelightHelpers;
import frc.utils.SwerveUtils;
import frc.utils.VisionUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private MAXSwerveModule m_frontLeft;

  private MAXSwerveModule m_frontRight;

  private MAXSwerveModule m_rearLeft;

  private MAXSwerveModule m_rearRight;

  // The gyro sensor
  private final AHRS m_gyro;
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

  public DriveSubsystem(AHRS gyro) {
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

    SmartDashboard.putBoolean("IMU_Connected", m_gyro.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("IMU_Angle", getAngle());
    SmartDashboard.putBoolean("Has Targed", LimelightHelpers.getTV(""));
    SmartDashboard.putNumber("tX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("tY", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("Target Angle To Turn : FR", VisionUtils.calculateAngle(m_gyro));
    SmartDashboard.putNumber("Distance From Target Range", VisionUtils.calculateDistance());
    if ((Math.abs(VisionUtils.calculateDistance()) <= 3) && (Math.abs(LimelightHelpers.getTX("")) <= 1)) {
      SmartDashboard.putBoolean("Ready To Shoot", true);
    } else {
      SmartDashboard.putBoolean("Ready To Shoot", false);
    }

    SmartDashboard.putBoolean("In Range", (Math.abs(VisionUtils.calculateDistance()) <= 3));
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
    final ChassisSpeeds vel = (fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngle()))
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
  }

  /** Zeroes the heading of the robot. */
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

  private Pose2d buildRelativePose(Pose2d relativeToRobot) {
    return getPose().plus(
        new Transform2d(
            new Pose2d(0, 0, new Rotation2d(0)),
            relativeToRobot));
  }

  public Trajectory buildRelativeTrajectory(List<Pose2d> seriesOfPoses) {
    List<Pose2d> relativePoses = seriesOfPoses.stream().map(
        this::buildRelativePose).collect(java.util.stream.Collectors.toList());
    return buildFieldTrajectory(relativePoses);
  }

  public Trajectory buildRelativeTrajectory(Pose2d endPose) {
    return buildRelativeTrajectory(endPose, List.of());
  }

  public Trajectory buildRelativeTrajectory(Pose2d endPose, List<Translation2d> pointsToNote) {
    return buildFieldTrajectory(buildRelativePose(endPose), pointsToNote);
  }

  public Trajectory buildFieldTrajectory(Pose2d endPose, List<Translation2d> transitionPoints) {
    Trajectory relativeTrajectory = TrajectoryGenerator.generateTrajectory(
        getPose(),
        transitionPoints,
        endPose,
        AutoConstants.kTrajectoryConfig);
    return relativeTrajectory;
  }

  public Trajectory BuildFieldTrajectory(Pose2d endPose) {
    return TrajectoryGenerator.generateTrajectory(
        getPose(),
        List.of(),
        endPose,
        AutoConstants.kTrajectoryConfig);
  }

  public Trajectory buildFieldTrajectory(List<Pose2d> routeToPose) {
    Trajectory relativeTrajectory = TrajectoryGenerator
        .generateTrajectory(routeToPose, AutoConstants.kTrajectoryConfig);
    return relativeTrajectory;

  }

  public Command drive(Trajectory objective) {

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new SwerveControllerCommand(
        objective,
        this::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        this::setModuleStates,
        this);

  }
}
