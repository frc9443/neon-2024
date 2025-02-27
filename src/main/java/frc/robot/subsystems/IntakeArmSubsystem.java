// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeArmConstants.kIntakeLiftMotorCanId, MotorType.kBrushless);
  private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder(IntakeArmConstants.kIntakeArmEncoderDioId);

  public IntakeArmSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Smart Dashboard output
    SmartDashboard.putNumber("IntakeArm position", getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveArm(double rate) {
    SmartDashboard.putNumber("Rate of Arm Movement", rate);
    m_IntakeArm.set(rate);
  }

  public void stopArm() {
    m_IntakeArm.stopMotor();
  }

  public double getPosition() {
    return m_ArmEncoder.getAbsolutePosition();
  }
  public boolean isArmOut(){
    if (getPosition() < .55) {
      return true;
    }
    return false;
  }
}
