// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkFlex m_shooterTopLeft = new CANSparkFlex(ShooterConstants.kTopLeftShooterMotorCanId,
      MotorType.kBrushless);
  private final CANSparkFlex m_shooterTopRight = new CANSparkFlex(ShooterConstants.kTopRightShooterMotorCanId,
      MotorType.kBrushless);
  private final CANSparkFlex m_shooterBottomLeft = new CANSparkFlex(ShooterConstants.kBottomLeftShooterMotorCanId,
      MotorType.kBrushless);
  private final CANSparkFlex m_shooterBottomRight = new CANSparkFlex(ShooterConstants.kBottomRightShooterMotorCanId,
      MotorType.kBrushless);
  private final RelativeEncoder m_BLShooterEncoder = m_shooterBottomLeft.getEncoder();
  private final RelativeEncoder m_TLShooterEncoder = m_shooterTopLeft.getEncoder();
  private final RelativeEncoder m_BRShooterEncoder = m_shooterBottomRight.getEncoder();
  private final RelativeEncoder m_TRShooterEncoder = m_shooterTopRight.getEncoder();

  private final Solenoid m_solenoid = new Solenoid(ShooterConstants.PneumaticHubCanId,
      PneumaticsModuleType.REVPH, ShooterConstants.SolenoidId);

  public ShooterSubsystem() {
    m_shooterBottomRight.setInverted(true);
    m_shooterTopRight.setInverted(false);
    m_shooterBottomLeft.setInverted(false);
    m_shooterTopLeft.setInverted(true);
  }

  public void Shoot(double upperVoltage, double lowerVoltage) {
    m_shooterTopLeft.setVoltage(upperVoltage);
    m_shooterTopRight.setVoltage(upperVoltage);
    m_shooterBottomLeft.setVoltage(lowerVoltage);
    m_shooterBottomRight.setVoltage(lowerVoltage);
  }

  public void Stop() {
    Shoot(0, 0);
  }

  public void doSolonoid(boolean isUp) {
    m_solenoid.set(isUp);
  }

  public boolean getIsShooterUp(){
    return m_solenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity Top Left", m_TLShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Velocity Top Right", m_TRShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Velocity Bottom Left", m_BLShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Velocity Bottom Right", m_BRShooterEncoder.getVelocity());
  }
}
