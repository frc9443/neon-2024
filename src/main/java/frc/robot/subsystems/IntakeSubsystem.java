// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
// import frc.utils.TCS34725_I2C;
// import frc.utils.TCS34725_I2C.TCSColor;
// import frc.utils.TCS34725_I2C.TransferAbortedException;

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
  // private final TCS34725_I2C m_ColorSensor = new TCS34725_I2C(false);
  private final RelativeEncoder m_IntakeLeftEncoder = m_IntakeLeft.getEncoder();
  private final RelativeEncoder m_IntakeRightEncoder = m_IntakeRight.getEncoder();
  DigitalInput m_leftLimitSwitch = new DigitalInput(IntakeConstants.kLeftLimitSwitchId);
  DigitalInput m_rightLimitSwitch = new DigitalInput(IntakeConstants.kRightLimitSwitchId);


  public IntakeSubsystem() {
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

    // try {
    // TCSColor color = m_ColorSensor.getRawData();
    // SmartDashboard.putNumber("ColorSensor R", color.getR());
    // SmartDashboard.putNumber("ColorSensor G", color.getG());
    // SmartDashboard.putNumber("ColorSensor B", color.getB());
    // double[] colors = {
    // color.getR(), color.getB(), color.getG()
    // };
    // SmartDashboard.putNumberArray("ColorSensor", colors);
    // SmartDashboard.putNumber("ColorSensor Temp",
    // TCS34725_I2C.calculateColorTemperature(color));
    // SmartDashboard.putNumber("ColorSensor LUX",
    // TCS34725_I2C.calculateLux(color));
    // } catch (TransferAbortedException ex) {
    // // noop
    // }
    SmartDashboard.putNumber("intake amperage left", m_IntakeLeft.getOutputCurrent());
    SmartDashboard.putNumber("intake amperage right", m_IntakeRight.getOutputCurrent());
    SmartDashboard.putNumber("Intake Velocity left", m_IntakeLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Velocity right", m_IntakeRightEncoder.getVelocity());

    SmartDashboard.putBoolean("Intake Has Note", hasNote());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void ingest(double voltage) {
    m_IntakeLeft.setVoltage(-voltage);
    m_IntakeRight.setVoltage(-voltage);
  }

  public void expel(double voltage) {
    ingest(-voltage);
  }

  public void stop() {
    m_IntakeLeft.stopMotor();
    m_IntakeRight.stopMotor();
  }

  public boolean hasNote() {
    return m_rightLimitSwitch.get() || m_leftLimitSwitch.get();
  }
}
