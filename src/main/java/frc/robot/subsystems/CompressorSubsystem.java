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
    m_compressor.enableAnalog(70, 90);
  }

  public double getPressure() {
    return m_compressor.getPressure();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
