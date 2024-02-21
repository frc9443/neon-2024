// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

  private final Talon m_IntakeArm = new Talon(IntakeArmConstants.kIntakeLiftMotorPWMId);
  private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder(IntakeArmConstants.kIntakeArmEncoderDioId);

  public IntakeArmSubsystem() {
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
    m_IntakeArm.set(rate);
  }

  public Command loadPosition() {
    return new PIDCommand(new PIDController(1, 0, 0),
        this::getPosition, 0.02, this::moveArm, this);
  }

  public Command shootPosition() {
    return new PIDCommand(new PIDController(1, 0, 0),
        this::getPosition, 0.6, this::moveArm, this);
  }

  public void stopArm() {
    m_IntakeArm.stopMotor();
  }

  public double getPosition() {
    return m_ArmEncoder.getAbsolutePosition();
  }
}
