package frc.robot.subsystems.intake_arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.utils.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class IntakeArm extends SubsystemBase {

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakeArm/Gains/kP", IntakeArmConstants.gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("IntakeArm/Gains/kI", IntakeArmConstants.gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakeArm/Gains/kD", IntakeArmConstants.gains.kD());
    private static final LoggedTunableNumber ff = new LoggedTunableNumber("IntakeArm/Gains/ff", IntakeArmConstants.ff);

    private final IntakeArmIO io;
    private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

    private final PIDController pid;

    public DoubleSupplier outThreshold = new LoggedTunableNumber("IntakeArm/OutThreshold", IntakeArmConstants.outThreshold);

    @RequiredArgsConstructor
    @Getter
    public enum Goal {
        LOAD(new LoggedTunableNumber("IntakeArm/LoadedPosition", IntakeArmConstants.loadPosition)),
        INTAKE(new LoggedTunableNumber("IntakeArm/IntakePosition", IntakeArmConstants.intakePosition)),
        AMP(new LoggedTunableNumber("IntakeArm/AmpPosition", IntakeArmConstants.ampPosition));

        private final DoubleSupplier armSetpointSupplier;
    }

    public enum Mode {
        TELEOP,
        GOAL,
    }

    @AutoLogOutput @Getter
    private Goal goal = Goal.LOAD;

    @AutoLogOutput @Getter
    private Mode mode = Mode.GOAL;

    @AutoLogOutput @Getter
    private double teleopInput = 0.0;

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
        pid = switch(Constants.getRobot()) {
            case NEON -> new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
            default -> new PIDController(0.0, 0.0, 0.0);
        };
        pid.setSetpoint(goal.getArmSetpointSupplier().getAsDouble());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeArm", inputs);

        switch(mode) {
            case TELEOP -> {
                if (MathUtil.isNear(0.0, teleopInput, 0.01)) {
                    io.stopArm();
                } else {
                    io.moveArm(teleopInput * 12.0);
                }
            }
            case GOAL -> {
                double output = pid.calculate(inputs.position);
                double voltage = (12.0 - ff.getAsDouble()) * output + Math.signum(output) * ff.getAsDouble();
                io.moveArm(MathUtil.clamp(voltage, -12.0, 12.0));
            }
            default -> io.stopArm();
        }
    }

    @AutoLogOutput
    public boolean isArmOut() {
        return (inputs.position > outThreshold.getAsDouble());
    }

    public void setGoal(Goal newGoal) {
        goal = newGoal;
        mode = Mode.GOAL;
        resetPID();
    }

    public void acceptTeleopInput(double x) {
        if (DriverStation.isTeleopEnabled()) {
            mode = Mode.TELEOP;
            teleopInput = x;
        }
    }

    public void stop() {
        mode = Mode.TELEOP;
        teleopInput = 0.0;
    }

    private void resetPID() {
        pid.reset();
        pid.setSetpoint(goal.getArmSetpointSupplier().getAsDouble());
        pid.disableContinuousInput();
        pid.setTolerance(0.05);
        pid.calculate(inputs.position);
    }
}
