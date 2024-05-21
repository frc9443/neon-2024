package frc.robot.subsystems.intake_arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeArmIOSim implements IntakeArmIO {

    private final double gearing = 1.0;
    private final static double armLengthMeters = 0.85;
    private static final double minRadians = IntakeArmConstants.intakePosition;
    private static final double maxRadians = IntakeArmConstants.loadPosition;
    private static final double startingRadians = IntakeArmConstants.ampPosition;


    // TODO: tune the sim config.  Completely random numbers for now...
    private SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            gearing,
            1.08, // jKgMetersSquared
            armLengthMeters,
            minRadians,
            maxRadians,
            false, // gravity
            startingRadians);

    private double appliedVolts = 0.0;

    public IntakeArmIOSim() {
        sim.setState(startingRadians, 0.0);
    }

    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.loopPeriodSeconds);

        inputs.position = sim.getAngleRads();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void moveArm(double voltage) {
        appliedVolts = voltage;
    }

    @Override
    public void stopArm() {
        appliedVolts = 0.0;
    }
}
