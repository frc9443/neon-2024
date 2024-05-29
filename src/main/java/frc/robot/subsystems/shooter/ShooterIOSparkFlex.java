package frc.robot.subsystems.shooter;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ShooterIOSparkFlex implements ShooterIO {

    private final CANSparkFlex topLeftMotor;
    private final CANSparkFlex topRightMotor;
    private final CANSparkFlex bottomLeftMotor;
    private final CANSparkFlex bottomRightMotor;

    private final RelativeEncoder bottomLeftEncoder;
    private final RelativeEncoder topLeftEncoder;
    private final RelativeEncoder bottomRightEncoder;
    private final RelativeEncoder topRightEncoder;

    private final Solenoid shootingAngleSolenoid;

    public ShooterIOSparkFlex() {
        topLeftMotor = new CANSparkFlex(ShooterConstants.topLeftCANId, MotorType.kBrushless);
        topRightMotor = new CANSparkFlex(ShooterConstants.topRightCANId, MotorType.kBrushless);
        bottomLeftMotor = new CANSparkFlex(ShooterConstants.bottomLeftCANId, MotorType.kBrushless);
        bottomRightMotor = new CANSparkFlex(ShooterConstants.bottomRightCANId, MotorType.kBrushless);

        topLeftEncoder = topLeftMotor.getEncoder();
        topRightEncoder = topRightMotor.getEncoder();
        bottomLeftEncoder = bottomLeftMotor.getEncoder();
        bottomRightEncoder = bottomRightMotor.getEncoder();

        List<CANSparkFlex> motors = Arrays.asList(topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor);

        motors.forEach(motor -> {
            motor.restoreFactoryDefaults();
            motor.setCANTimeout(250);
            motor.setSmartCurrentLimit(30);
            motor.enableVoltageCompensation(12.0);
            motor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        });

        // Set appropriate inversion for left/right motors
        topLeftMotor.setInverted(true);
        topRightMotor.setInverted(false);
        bottomLeftMotor.setInverted(false);
        bottomRightMotor.setInverted(true);

        Arrays.asList(topLeftEncoder, topRightEncoder, bottomLeftEncoder, bottomRightEncoder).forEach(encoder -> {
            encoder.setMeasurementPeriod(10);
            encoder.setAverageDepth(2);
        });

        motors.forEach(motor -> motor.burnFlash());

        shootingAngleSolenoid = new Solenoid(
            ShooterConstants.PneumaticHubCanId,
            PneumaticsModuleType.REVPH,
            ShooterConstants.SolenoidId);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.appliedVoltsTopLeft = topLeftMotor.getAppliedOutput();
        inputs.currentAmpsTopLeft = topLeftMotor.getOutputCurrent();
        inputs.velocityTopLeft = topLeftEncoder.getVelocity();

        inputs.appliedVoltsBottomLeft = bottomLeftMotor.getAppliedOutput();
        inputs.currentAmpsBottomLeft = bottomLeftMotor.getOutputCurrent();
        inputs.velocityBottomLeft = bottomLeftEncoder.getVelocity();

        inputs.appliedVoltsBottomRight = bottomRightMotor.getAppliedOutput();
        inputs.currentAmpsBottomRight = bottomRightMotor.getOutputCurrent();
        inputs.velocityBottomRight = bottomRightEncoder.getVelocity();

        inputs.appliedVoltsTopRight = topRightMotor.getAppliedOutput();
        inputs.currentAmpsTopRight = topRightMotor.getOutputCurrent();
        inputs.velocityTopRight = topRightEncoder.getVelocity();
    }

    public void shoot(double upperVoltage, double lowerVoltage) {
        topLeftMotor.setVoltage(upperVoltage);
        topRightMotor.setVoltage(upperVoltage);
        bottomLeftMotor.setVoltage(lowerVoltage);
        bottomRightMotor.setVoltage(lowerVoltage);
    }

    public void stop() {
        shoot(0, 0);
    }

    public void setAngle(ShooterAngle position) {
        shootingAngleSolenoid.set(position == ShooterAngle.HIGH);
    }
}
