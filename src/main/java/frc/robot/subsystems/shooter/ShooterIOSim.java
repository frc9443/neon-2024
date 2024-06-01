package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

    private final DCMotorSim topSim;
    private final DCMotorSim bottomSim;

    private double appliedTopVoltage = 0.0;
    private double appliedBottomVoltage = 0.0;

    public ShooterIOSim() {
        DCMotor topMotor = DCMotor.getNeo550(2);
        DCMotor bottomMotor = DCMotor.getNeo550(2);
        topSim = new DCMotorSim(topMotor, 1.0, 0.001);
        bottomSim = new DCMotorSim(bottomMotor, 1.0, 0.001);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        topSim.update(Constants.loopPeriodSeconds);
        bottomSim.update(Constants.loopPeriodSeconds);

        inputs.appliedVoltsTopLeft = inputs.appliedVoltsTopRight = appliedTopVoltage;
        inputs.currentAmpsTopLeft = inputs.currentAmpsTopRight = topSim.getCurrentDrawAmps();
        inputs.velocityTopLeft = inputs.velocityTopRight = topSim.getAngularVelocityRadPerSec();

        inputs.appliedVoltsBottomLeft = inputs.appliedVoltsBottomRight = appliedBottomVoltage;
        inputs.currentAmpsBottomLeft = inputs.currentAmpsBottomRight = bottomSim.getCurrentDrawAmps();
        inputs.velocityBottomLeft = inputs.velocityBottomRight = bottomSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void shoot(double upperVoltage, double lowerVoltage) {
        appliedTopVoltage = upperVoltage;
        topSim.setInputVoltage(appliedTopVoltage);
        appliedBottomVoltage = lowerVoltage;
        bottomSim.setInputVoltage(appliedBottomVoltage);
    }
    
    @Override
    public void stop() {
        shoot(0.0, 0.0);
    }
}
