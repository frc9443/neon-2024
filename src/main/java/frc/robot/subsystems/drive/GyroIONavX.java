package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class GyroIONavX implements GyroIO {
    private AHRS gyro = null;

    public GyroIONavX() {
        switch (Constants.getRobot()) {
            case NEON -> gyro = new AHRS(SerialPort.Port.kUSB);
            case HELIUM -> gyro = new AHRS(SerialPort.Port.kMXP);
            default -> gyro = null;
        }
        if (gyro != null) {
            gyro.reset();
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if (gyro == null) {
            inputs.connected = false;
            return;
        }
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = new Rotation2d(-Math.toRadians(gyro.getAngle()));
        inputs.yawVelocityRadPerSec = Math.toRadians(gyro.getRate());
    }

    @Override
    public void reset() {
        if (gyro != null) {
            gyro.reset();
        }
    }

}
