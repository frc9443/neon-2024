package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsIORev implements PneumaticsIO {

    private final Compressor compressor;
    private final Solenoid shootingAngleSolenoid;

    private double currentMin = 0;
    private double currentMax = 0;

    public PneumaticsIORev() {
        compressor = new Compressor(PneumaticsConstants.PneumaticHubCanId, PneumaticsModuleType.REVPH);
        compressor.disable();

        shootingAngleSolenoid = new Solenoid(
                PneumaticsConstants.PneumaticHubCanId,
                PneumaticsModuleType.REVPH,
                PneumaticsConstants.SolenoidId);
    }

    public void updateInputs(PneumaticsIOInputs inputs) {
        inputs.isEnabled = compressor.isEnabled();
        inputs.isFull = compressor.getPressureSwitchValue();
        inputs.pressure = compressor.getPressure();
        inputs.appliedCurrent = compressor.getCurrent();
        inputs.appliedVolts = compressor.getAnalogVoltage();
        inputs.minPressure = currentMin;
        inputs.maxPressure = currentMax;

        inputs.shooterAngle = shootingAngleSolenoid.get() ? ShooterAngle.LOW : ShooterAngle.HIGH;
    }

    public void setPressure(double min, double max) {
        if (min != currentMin || max != currentMax) {
            compressor.enableAnalog(currentMin = min, currentMax = max);
        }
    }

    public void setShooterAngle(ShooterAngle position) {
        shootingAngleSolenoid.set(position == ShooterAngle.HIGH);
    }

    public void stop() {
        compressor.disable();
    }
}
