// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double loopPeriodSeconds = 0.02; // 20ms loop

  private static RobotType robotType = RobotType.NEON;
  public static final boolean tuningMode = false;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIM) {
      robotType = RobotType.NEON;
    }
    return robotType;
  }

  public static Mode getMode() {
    if (getRobot() == RobotType.SIM) {
      return Mode.SIM;
    }
    return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
  }

  public static enum RobotType {
    NEON,
    HELIUM,
    DEV,
    SIM
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; // 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMagnitudeSlewRate =  2.7 * kMaxSpeedMetersPerSecond; // meters per second^2 1.8
    public static final double kRotationalSlewRate = 2.0 * kMaxAngularSpeed; // radians per second^2

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = true;

    // SPARK MAX CAN IDs Top 8 are Neon
    public static final class Neon {
      public static final int kFrontLeftDrivingCanId = 6;
      public static final int kRearLeftDrivingCanId = 4;
      public static final int kFrontRightDrivingCanId = 8;
      public static final int kRearRightDrivingCanId = 2;

      public static final int kFrontLeftTurningCanId = 5;
      public static final int kRearLeftTurningCanId = 3;
      public static final int kFrontRightTurningCanId = 7;
      public static final int kRearRightTurningCanId = 9;
    }

    public static final class Helium {
      public static final int kFrontLeftDrivingCanId = 8;
      public static final int kRearLeftDrivingCanId = 2;
      public static final int kFrontRightDrivingCanId = 6;
      public static final int kRearRightDrivingCanId = 4;

      public static final int kFrontLeftTurningCanId = 9;
      public static final int kRearLeftTurningCanId = 3;
      public static final int kFrontRightTurningCanId = 7;
      public static final int kRearRightTurningCanId = 5;
    }
  }

  public static final class IntakeConstants {
    public static final int kIntakeRightRollerCanId = 32;
    public static final int kIntakeLeftRollerCanId = 31;
    public static final int kLeftLimitSwitchId = 8;
    public static final int kRightLimitSwitchId = 0;
  }
  public static final class BlinkinConstants {
    public static final int kBlinkinPWMId = 0;
    public static final double cBreathRed = -0.17;
    public static final double cBreathBlue = -0.15;
    public static final double cStrobeRed = -0.11;
    public static final double cStrobeBlue = -0.09;
    public static final double cStrobeGold = -0.07;
    public static final double cSolidGold = 0.67;
    public static final double cRainbow = -0.99;

  }

  public static final class ClimberConstants {
    public static final int kLeftClimberMotorCanId = 41;
    public static final int kRightClimberMotorCanId = 42;
  }

  public static final class ShooterConstants {
    public static final int PneumaticHubCanId = 20;
    public static final DutyCycleEncoder m_encoder = new DutyCycleEncoder(9);
    public static final int kTopLeftShooterMotorCanId = 21;
    public static final int kTopRightShooterMotorCanId = 22;
    public static final int kBottomLeftShooterMotorCanId = 23;
    public static final int kBottomRightShooterMotorCanId = 24;
    public static final int SolenoidId = 1;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kColorControllerPort = 5;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class VisionConstants {
    public static final double heightOfCamera = .254;
    public static final double heightOfCenterSpeaker = 1.45;
    public static final double angleOfCamera = Math.toRadians(19);
    public static final double targetDistanceToShoot = 97;
    // public static final double idealShootingDistance = 2.5;
    public static final double shootingDistanceVariance = 0.1;
    public static final double shootingAngleVariance = 1.5;

  }

  public static final class GyroConstants {
    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 1;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second orgininal 10
    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;
  }
  public static final class ArmConstants{
    public static final double kTurnP = .7;
    public static final double kTurnI = .8;
    public static final double kTurnD = .05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.8;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  // Entrypoint for gradle to sanity-check robot type before deploying
  public static void main(String... args) {
    if (robotType == RobotType.SIM) {
      System.err.println("Cannot deploy, SIM robot selected!");
      System.exit(1);
    }
  }

}