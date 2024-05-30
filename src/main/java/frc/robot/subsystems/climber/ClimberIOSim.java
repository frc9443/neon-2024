package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO{

    private final DCMotorSim leftSim; // TODO: consider using ElevatorSim instead
    private final DCMotorSim rightSim;

    private double appliedLeftVoltage = 0.0;
    private double appliedRightVoltage = 0.0;

    public ClimberIOSim() {
        DCMotor leftMotor = DCMotor.getNEO(1);
        DCMotor rightMotor = DCMotor.getNEO(1);
        leftSim = new DCMotorSim(leftMotor, 1.0, 0.001);
        rightSim = new DCMotorSim(rightMotor, 1.0, 0.001);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.appliedVoltsLeft = appliedLeftVoltage;
        inputs.currentAmpsLeft = leftSim.getCurrentDrawAmps();
        inputs.velocityLeft = leftSim.getAngularVelocityRadPerSec();

        inputs.appliedVoltsRight = appliedRightVoltage;
        inputs.currentAmpsRight = rightSim.getCurrentDrawAmps();
        inputs.velocityRight = rightSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void climb(double leftRate, double rightRate) {
        appliedLeftVoltage = leftRate;
        leftSim.setInputVoltage(appliedLeftVoltage);
        appliedRightVoltage = rightRate;
        rightSim.setInputVoltage(appliedRightVoltage);
    }

    @Override
    public void stop() {
        climb(0.0, 0.0);
    }

}
