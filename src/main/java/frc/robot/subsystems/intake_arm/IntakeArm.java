package frc.robot.subsystems.intake_arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.utils.LoggedTunableNumber;

import java.util.Random;
import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class IntakeArm extends SubsystemBase {

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakeArm/Gains/kP", IntakeArmConstants.gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("IntakeArm/Gains/kI", IntakeArmConstants.gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakeArm/Gains/kD", IntakeArmConstants.gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("IntakeArm/Gains/ff", IntakeArmConstants.gains.kS());

    private final IntakeArmIO io;
    private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

    private final PIDController pid;
    private final SimpleMotorFeedforward ff;

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

    private final IntakeArmMechanism actualMechanism;
    private final IntakeArmMechanism goalMechanism;

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
        pid = switch(Constants.getRobot()) {
            case NEON -> new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
            // case SIM -> new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
            default -> new PIDController(1.0, 0.0, 0.0);
        };
        pid.setSetpoint(goal.getArmSetpointSupplier().getAsDouble());
        ff = new SimpleMotorFeedforward(kS.getAsDouble(), 1.0);
        actualMechanism = new IntakeArmMechanism("actual", Color.kBlueViolet);
        goalMechanism = new IntakeArmMechanism("goal", Color.kGreen);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeArm", inputs);

        switch(mode) {
            case TELEOP -> {
                if (MathUtil.isNear(0.0, teleopInput, 0.01)) {
                    // Stop moving if input in dead zone
                    io.stopArm();
                } else {
                    io.moveArm(teleopInput * 12.0);
                }
            }
            case GOAL -> {
                double output = pid.calculate(inputs.position);
                if (atGoal()) {
                    io.stopArm();
                } else {
                    double voltage = ff.calculate(output);
                    io.moveArm(MathUtil.clamp(voltage, -12.0, 12.0));
                }
            }
            default -> io.stopArm();
        }
        actualMechanism.update(inputs.position);
        goalMechanism.update(goal.getArmSetpointSupplier().getAsDouble());
    }

    @AutoLogOutput
    public boolean isOut() {
        return (inputs.position < outThreshold.getAsDouble());
    }

    @AutoLogOutput
    public boolean atGoal() {
        return pid.atSetpoint();
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

    public Command intake() {
       return doGoal(Goal.INTAKE);
    }

    public Command load() {
        return doGoal(Goal.LOAD);
    }

    public Command amp() {
        return doGoal(Goal.AMP);
    }

    private Command doGoal(Goal goal) {
        return new FunctionalCommand(
                () -> setGoal(goal),
                () -> {},
                (interrupted) -> stop(),
                this::atGoal,
                this);
    }

    private void resetPID() {
        pid.reset();
        pid.setSetpoint(goal.getArmSetpointSupplier().getAsDouble());
        pid.disableContinuousInput();
        pid.setTolerance(0.05);
        pid.calculate(inputs.position);
    }
}
