// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn
 * motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (Constants.getRobot()) {
      case NEON -> {

        switch (index) { // FL, FR, BL, BR
          case 0 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Neon.kFrontLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Neon.kFrontLeftTurningCanId, MotorType.kBrushless);
          }
          case 1 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Neon.kFrontRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Neon.kFrontRightTurningCanId, MotorType.kBrushless);
          }
          case 2 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Neon.kRearLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Neon.kRearLeftTurningCanId, MotorType.kBrushless);
          }
          case 3 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Neon.kRearRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Neon.kRearRightTurningCanId, MotorType.kBrushless);
          }
          default -> throw new RuntimeException("Invalid module index");
        }
      }
      case HELIUM -> {
        switch (index) {
          case 0 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Helium.kFrontLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Helium.kFrontLeftTurningCanId, MotorType.kBrushless);
          }
          case 1 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Helium.kFrontRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Helium.kFrontRightTurningCanId, MotorType.kBrushless);
          }
          case 2 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Helium.kRearLeftDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Helium.kRearLeftTurningCanId, MotorType.kBrushless);
          }
          case 3 -> {
            driveSparkMax = new CANSparkMax(DriveConstants.Helium.kRearRightDrivingCanId, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.Helium.kRearRightTurningCanId, MotorType.kBrushless);
          }
          default -> throw new RuntimeException("Invalid module index");
        }
      }
      default -> throw new RuntimeException("Robot " + Constants.getRobot() + " does not use SparkMax modules");
    }

    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderOffset = new Rotation2d(0.0);

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(50);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);
    driveSparkMax.setIdleMode(IdleMode.kBrake);
    turnSparkMax.setIdleMode(IdleMode.kBrake);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnAbsoluteEncoder.setPositionConversionFactor(Math.PI * 2.0);
    turnAbsoluteEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkMaxOdometryThread.getInstance()
        .registerSignal(
            () -> {
              double value = driveEncoder.getPosition();
              if (driveSparkMax.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
              } else {
                return OptionalDouble.empty();
              }
            });
    turnPositionQueue = SparkMaxOdometryThread.getInstance()
        .registerSignal(
            () -> {
              double value = turnAbsoluteEncoder.getPosition();
              if (turnSparkMax.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
              } else {
                return OptionalDouble.empty();
              }
            });

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
        / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] { driveSparkMax.getOutputCurrent() };

    inputs.turnAbsolutePosition = new Rotation2d(turnAbsoluteEncoder.getPosition() - Math.PI)
        .minus(absoluteEncoderOffset);
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnAbsoluteEncoder.getVelocity());
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
        .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRotations(value))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
