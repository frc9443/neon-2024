package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim sim;
    private double appliedVoltage = 0.0;

    public IntakeIOSim() {
        DCMotor motorModel = DCMotor.getNEO(1);
        sim = new DCMotorSim(motorModel, 1.0, 0.001);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        sim.update(Constants.loopPeriodSeconds);

        inputs.appliedVoltsLeft = inputs.appliedVoltsRight = appliedVoltage;
        inputs.currentAmpsLeft = inputs.currentAmpsRight = sim.getCurrentDrawAmps();
        inputs.velocityLeft = inputs.velocityRight = sim.getAngularVelocityRadPerSec();
    }

    @Override
    public void ingest(double voltage) {
        appliedVoltage = voltage;
        sim.setInputVoltage(voltage);
    }

    @Override
    public void expel(double voltage) {
        ingest(-voltage);
    }

    @Override
    public void stop() {
        ingest(0.0);
    }
    
}
