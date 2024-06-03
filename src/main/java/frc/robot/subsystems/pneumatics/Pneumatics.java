package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LoggedTunableNumber;

public class Pneumatics extends SubsystemBase {

    private final PneumaticsIO io;
    private final PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();

    private static final LoggedTunableNumber minPressure = new LoggedTunableNumber("Pneumatics/MinPressure", 35);
    private static final LoggedTunableNumber enoughPressure = new LoggedTunableNumber("Pneumatics/EnoughPressure", 45);
    private static final LoggedTunableNumber maxPressure = new LoggedTunableNumber("Pneumatics/MaxPressure", 90);

    public Pneumatics(PneumaticsIO io) {
        this.io = io;
        io.setPressure(minPressure.getAsDouble(), maxPressure.getAsDouble());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pneumatics", inputs);

        if (minPressure.hasChanged(hashCode()) || maxPressure.hasChanged(hashCode())) {
            io.setPressure(minPressure.getAsDouble(), maxPressure.getAsDouble());
        }
    }

    public void setShooterAngle(PneumaticsIO.ShooterAngle position) {
        io.setShooterAngle(position);
    }

    @AutoLogOutput
    public double getPressure() {
        return inputs.pressure;
    }

    @AutoLogOutput
    public boolean isShooterUp() {
        return inputs.shooterAngle == PneumaticsIO.ShooterAngle.HIGH;
    }

    public Command setShooterAngleCommand(PneumaticsIO.ShooterAngle targetAngle) {
        return runOnce(() -> setShooterAngle(targetAngle));
    }

    public Command ensurePressureCommand() {
        return Commands.waitUntil(() -> getPressure() > enoughPressure.getAsDouble());
    }

}
