// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_IntakeLeft = new CANSparkMax(IntakeConstants.kIntakeLeftRollerCanId,
      MotorType.kBrushless);
  private final CANSparkMax m_IntakeRight = new CANSparkMax(IntakeConstants.kIntakeRightRollerCanId,
      MotorType.kBrushless);
  private final RelativeEncoder m_IntakeLeftEncoder = m_IntakeLeft.getEncoder();
  private final RelativeEncoder m_IntakeRightEncoder = m_IntakeRight.getEncoder();
  DigitalInput m_leftLimitSwitch = new DigitalInput(IntakeConstants.kLeftLimitSwitchId);
  DigitalInput m_rightLimitSwitch = new DigitalInput(IntakeConstants.kRightLimitSwitchId);

  private boolean intakeActive = false;

  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake amperage left", m_IntakeLeft.getOutputCurrent());
    SmartDashboard.putNumber("intake amperage right", m_IntakeRight.getOutputCurrent());
    SmartDashboard.putNumber("Intake Velocity left", m_IntakeLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Velocity right", m_IntakeRightEncoder.getVelocity());

    SmartDashboard.putBoolean("Intake Has Note", hasNote());
  }

  public void ingest() {
    ingest(IntakeConstants.kIntakeVoltage);
  }

  public void ingest(double voltage) {
    intakeActive = true;
    m_IntakeLeft.setVoltage(-voltage);
    m_IntakeRight.setVoltage(-voltage);
  }

  public void expel(double voltage) {
    ingest(-voltage);
  }

  public void stop() {
    intakeActive = false;
    m_IntakeLeft.stopMotor();
    m_IntakeRight.stopMotor();
  }

  public boolean hasNote() {
    return !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
  }

  public boolean isIntakeActive(){
    return intakeActive;
  }
  
}
