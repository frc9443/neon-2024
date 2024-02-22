// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final Solenoid m_solenoid = new Solenoid(ShooterConstants.PneumaticHubCanId,
      PneumaticsModuleType.REVPH, ShooterConstants.SolenoidId);

  public ShooterSubsystem() {
    m_shooterBottomRight.setInverted(true);
    m_shooterTopRight.setInverted(false);
    m_shooterBottomLeft.setInverted(false);
    m_shooterTopLeft.setInverted(true);
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

  public void Shoot(double upperRate, double lowerRate) {
    m_shooterTopLeft.set(upperRate);
    m_shooterTopRight.set(upperRate);
    m_shooterBottomLeft.set(lowerRate);
    m_shooterBottomRight.set(lowerRate);
  }

  public void Stop() {
    m_shooterTopLeft.setVoltage(0);
    m_shooterTopRight.setVoltage(0);
    m_shooterBottomLeft.setVoltage(0);
    m_shooterBottomRight.setVoltage(0);
  }

  public void doSolonoid(boolean isUp) {
    m_solenoid.set(isUp);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
