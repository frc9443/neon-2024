/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {

  /*
   * Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor.
   * -1 corresponds to 1000us
   * 0 corresponds to 1500us
   * +1 corresponds to 2000us
   */
  private static Spark m_blinkin = null;
  private VisionSubsystem m_VisionSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  private IntakeArmSubsystem m_IntakeArmSubsystem;

  private boolean inPositition = false;
  private boolean hasNote = false;
  private boolean armOut = false;
  private boolean intakeActive = false;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public BlinkinSubsystem(VisionSubsystem vSubsystem, IntakeSubsystem iSubsystem,
               IntakeArmSubsystem iArmSubsystem) {
    m_blinkin = new Spark(0);
    m_VisionSubsystem = vSubsystem;
    m_IntakeSubsystem = iSubsystem;
    m_IntakeArmSubsystem = iArmSubsystem;
    setColor(BlinkinConstants.cDarkGreen);
  }

  @Override
  public void periodic() {
    setColor(checkForColor());
  }
  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to
   * patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */

  public void setColor(double color) {
    m_blinkin.set(color);
  }

  public double checkForColor() {
    inPositition = m_VisionSubsystem.lockedOn();
    hasNote = m_IntakeSubsystem.hasNote();
    intakeActive = m_IntakeSubsystem.isIntakeActive();
    armOut = m_IntakeArmSubsystem.isArmOut();

    if (inPositition && hasNote)
      return BlinkinConstants.cRainbow;
    else if (hasNote)
      return BlinkinConstants.cDarkBlue;
    else if(armOut && intakeActive)
      return BlinkinConstants.cYellow;
    else if(armOut)
      return BlinkinConstants.cOrange;
    else
      return BlinkinConstants.cDarkRed;
    
  }
}