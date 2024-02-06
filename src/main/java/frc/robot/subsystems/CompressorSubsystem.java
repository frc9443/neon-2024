// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class CompressorSubsystem extends SubsystemBase {
public final Compressor m_compressor = new Compressor(ShooterConstants.PneumaticHubCanId, PneumaticsModuleType.REVPH);
   
  
  public CompressorSubsystem() {
    SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
  }

public void doCompressor(boolean toggleCompressor){
    // On button press, toggle the compressor.
    if (toggleCompressor) {
      // Check whether the compressor is currently enabled.
      boolean isCompressorEnabled = m_compressor.isEnabled();
      if (isCompressorEnabled) {
        // Disable closed-loop mode on the compressor.
        m_compressor.disable();
      } else {
        m_compressor.enableAnalog(70, 90);
      }
    }
  }
public double getPressure(){
    return m_compressor.getPressure();
}
public BooleanSupplier morePressureNeeded(){
    m_compressor.enableAnalog(70, 90);
    SmartDashboard.putBoolean("Compresser On", m_compressor.isEnabled());
    
    return ()->{
        if(getPressure() < 70)
        {
            return true;
        }
        return false;
    };
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
