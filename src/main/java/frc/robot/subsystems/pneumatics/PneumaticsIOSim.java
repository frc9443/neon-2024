package frc.robot.subsystems.pneumatics;

public class PneumaticsIOSim implements PneumaticsIO {

    private boolean isEnabled = false;
    private boolean rising = true;
    private double minPressure = 0.0;
    private double maxPressure = 0.0;
    private double pressure = 0.0;
    private ShooterAngle shooterAngle = ShooterAngle.HIGH;

    public void updateInputs(PneumaticsIOInputs inputs) {
        simulateCompressor();

        inputs.isEnabled = isEnabled;
        inputs.isFull = isEnabled;
        inputs.pressure = pressure;
        inputs.appliedCurrent = 0;
        inputs.appliedVolts = 0;
        inputs.minPressure = minPressure;
        inputs.maxPressure = maxPressure;

        inputs.shooterAngle = shooterAngle;
    }

    @Override
    public void setPressure(double min, double max) {
        minPressure = min;
        maxPressure = max;
    }
    
    @Override
    public void setShooterAngle(ShooterAngle position) {
        if (shooterAngle != position && position == ShooterAngle.HIGH) {
            // Simulate loss of pressure when raising shooter
            pressure = Math.max(0, pressure - 1.0);
        }
        shooterAngle = position;
    }

    @Override
    public void stop() {
        isEnabled = false;
        minPressure = 0;
        maxPressure = 0;
    }

    // Simple simulation of a leaky compressor
    private void simulateCompressor() {
        if (isEnabled) {
            if (rising) {
                if (pressure < maxPressure) {
                    pressure += 0.02;
                } else {
                    rising = false;
                }
            } else {
                if (pressure > minPressure) {
                    pressure -= 0.01;
                } else {
                    rising = true;
                }
            }
        } else {
            pressure = Math.max(0, pressure - 0.01);
        }
    }
    
}
