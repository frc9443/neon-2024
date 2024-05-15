package frc.robot.subsystems.intake_arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeArmIOSparkMax implements IntakeArmIO {

    private final CANSparkMax motor = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(9);

    public IntakeArmIOSparkMax() {
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);
        motor.burnFlash();
    }

    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
        inputs.position = encoder.getAbsolutePosition();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void moveArm(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopArm() {
        motor.stopMotor();
    }

}
