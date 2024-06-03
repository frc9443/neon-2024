package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; // 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMagnitudeSlewRate = 2.7 * kMaxSpeedMetersPerSecond; // meters per second^2 1.8
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
