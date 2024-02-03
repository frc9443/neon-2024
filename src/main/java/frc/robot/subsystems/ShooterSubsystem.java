// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkFlex m_shooterTopLeft = new CANSparkFlex(ShooterConstants.kTopLeftShooterMotorCanId, MotorType.kBrushless);
    private final CANSparkFlex m_shooterTopRight = new CANSparkFlex(ShooterConstants.kTopRightShooterMotorCanId, MotorType.kBrushless); 
    private final CANSparkFlex m_shooterBottomLeft = new CANSparkFlex(ShooterConstants.kBottomLeftShooterMotorCanId, MotorType.kBrushless); 
    private final CANSparkFlex m_shooterBottomRight = new CANSparkFlex(ShooterConstants.kBottomRightShooterMotorCanId, MotorType.kBrushless);

    public ShooterSubsystem() {
        m_shooterBottomRight.setInverted(true);
        m_shooterTopRight.setInverted(true);
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public void Shoot()
  {
    m_shooterTopLeft.setVoltage(11);
    m_shooterTopRight.setVoltage(11);
    m_shooterBottomLeft.setVoltage(5.5);
    m_shooterBottomRight.setVoltage(5.5);
  }
  public void Stop()
  {
    m_shooterTopLeft.setVoltage(0); 
    m_shooterTopRight.setVoltage(0);  
    m_shooterBottomLeft.setVoltage(0);
    m_shooterBottomRight.setVoltage(0);
  }

  public void doSolonoid(boolean toggleSolonoid) {
    ShooterConstants.m_solenoid.set(toggleSolonoid);
  }

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
}
