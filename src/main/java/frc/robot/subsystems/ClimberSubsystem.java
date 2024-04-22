// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax m_LeftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorCanId,
      MotorType.kBrushless);
  private CANSparkMax m_RightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorCanId,
      MotorType.kBrushless);

  public ClimberSubsystem() {
  }

  public void moveClimber(double rate) {
    this.moveClimber(rate, rate);
  }

  public void moveClimber(double leftClimberRate, double rightClimberRate) {
    m_LeftClimberMotor.set(-leftClimberRate);
    m_RightClimberMotor.set(-rightClimberRate);
  }

  public void stopClimber() {
    m_LeftClimberMotor.stopMotor();
    m_RightClimberMotor.stopMotor();
  }

}
