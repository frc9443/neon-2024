package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax leftRoller;
    private final CANSparkMax rightRoller;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    DigitalInput leftSwitch;
    DigitalInput rightSwitch;

    public IntakeIOSparkMax() {
        leftRoller = new CANSparkMax(IntakeConstants.kIntakeLeftRollerCanId, CANSparkMax.MotorType.kBrushless);
        rightRoller = new CANSparkMax(IntakeConstants.kIntakeRightRollerCanId, CANSparkMax.MotorType.kBrushless);

        leftEncoder = leftRoller.getEncoder();
        rightEncoder = rightRoller.getEncoder();

        leftSwitch = new DigitalInput(IntakeConstants.kLeftLimitSwitchId);
        rightSwitch = new DigitalInput(IntakeConstants.kRightLimitSwitchId);

        leftRoller.restoreFactoryDefaults();
        rightRoller.restoreFactoryDefaults();

        leftRoller.setCANTimeout(250);
        rightRoller.setCANTimeout(250);

        leftRoller.setInverted(false);
        rightRoller.setInverted(false);

        leftRoller.setSmartCurrentLimit(30);
        rightRoller.setSmartCurrentLimit(30);
        leftRoller.enableVoltageCompensation(12.0);
        rightRoller.enableVoltageCompensation(12.0);

        leftEncoder.setMeasurementPeriod(10);
        rightEncoder.setMeasurementPeriod(10);
        leftEncoder.setAverageDepth(2);
        rightEncoder.setAverageDepth(2);

        leftRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);

        leftRoller.burnFlash();
        rightRoller.burnFlash();

        leftRoller.setCANTimeout(0);
        rightRoller.setCANTimeout(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityLeft = leftEncoder.getVelocity();
        inputs.appliedVoltsLeft = leftRoller.getAppliedOutput();
        inputs.currentAmpsLeft = leftRoller.getOutputCurrent();

        inputs.velocityRight = rightEncoder.getVelocity();
        inputs.appliedVoltsRight = rightRoller.getAppliedOutput();
        inputs.currentAmpsRight = rightRoller.getOutputCurrent();

        inputs.hasNote = leftSwitch.get() || rightSwitch.get();
    }

    @Override
    public void ingest(double voltage) {
        leftRoller.setVoltage(voltage);
        rightRoller.setVoltage(voltage);
    }

    @Override
    public void expel(double voltage) {
        leftRoller.setVoltage(-voltage);
        rightRoller.setVoltage(-voltage);
    }

    @Override
    public void stop() {
        leftRoller.stopMotor();
        rightRoller.stopMotor();
    }
}