// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import frc.robot.subsystems.drive.DriveConstants;

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
  public static ControlMode controlMode = ControlMode.Neon_2024_PostSeason;
  public static final boolean tuningMode = true;

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

  public static enum ControlMode {
    Neon_2024_Competition,
    Neon_2024_PostSeason,
    Neon_2024_Demo,
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kColorControllerPort = 5;
    public static final double kDriveDeadband = 0.05;
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