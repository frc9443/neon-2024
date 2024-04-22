// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class CompressorSubsystem extends SubsystemBase {
    public final Compressor m_compressor = new Compressor(ShooterConstants.PneumaticHubCanId, PneumaticsModuleType.REVPH);

    public CompressorSubsystem() {
        m_compressor.enableAnalog(35, 90);
    }

    public double getPressure() {
        return m_compressor.getPressure();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
    }
}
