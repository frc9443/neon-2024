// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_IntakeArm = new CANSparkMax(IntakeConstants.kIntakeLeftRollerCanId, MotorType.kBrushless);
  private final Encoder m_ArmEncoder = new 
  Encoder(IntakeArmConstants.kIntakeArmEncoderADioId, IntakeArmConstants.kIntakeArmEncoderBDioId);


  public IntakeArmSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void moveArm(double rate)
  {
    m_IntakeArm.set(rate);
  }
  public void stopArm()
  {
    m_IntakeArm.stopMotor();
  }
  public double getDistance()
  {
    return m_ArmEncoder.getDistance();
  }
}
