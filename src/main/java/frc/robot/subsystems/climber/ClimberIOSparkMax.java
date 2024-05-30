package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOSparkMax implements ClimberIO {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    public ClimberIOSparkMax() {
        leftMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorCanId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorCanId, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setCANTimeout(250);
        rightMotor.setCANTimeout(250);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);

        leftMotor.enableVoltageCompensation(12.0);
        rightMotor.enableVoltageCompensation(12.0);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        leftMotor.setCANTimeout(0);
        rightMotor.setCANTimeout(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.velocityLeft = leftMotor.getEncoder().getVelocity();
        inputs.appliedVoltsLeft = leftMotor.getAppliedOutput();
        inputs.currentAmpsLeft = leftMotor.getOutputCurrent();
        inputs.velocityRight = rightMotor.getEncoder().getVelocity();
        inputs.appliedVoltsRight = rightMotor.getAppliedOutput();
        inputs.currentAmpsRight = rightMotor.getOutputCurrent();
    }

    @Override
    public void climb(double leftRate, double rightRate) {
        leftMotor.set(leftRate);
        rightMotor.set(rightRate);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
