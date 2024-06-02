package frc.robot.subsystems.leds;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake_arm.IntakeArm;
import frc.robot.subsystems.vision.Vision;
import frc.utils.VirtualSubsystem;

public class Leds extends VirtualSubsystem {

    private final LedsIO io;

    private LedsIO.ColorPattern currentPattern = null;

    private Intake intake;
    private IntakeArm intakeArm;
    private Vision vision;

    public Leds(LedsIO io) {
        this.io = io;
    }

    public void setColorPattern(LedsIO.ColorPattern pattern) {
        io.setColorPattern(pattern);
    }

    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    public void setIntakeArm(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
    }

    public void setVision(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void periodic() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        boolean blueAlliance = !(ally.isPresent() && ally.get() == Alliance.Red);
        boolean hasNote = intake != null && intake.hasNote();
        boolean armOut = intake != null && intakeArm.isOut();
        boolean lockedOn = vision != null && vision.lockedOn();

        LedsIO.ColorPattern pattern;

        if (lockedOn && hasNote) {
            pattern = LedsIO.ColorPattern.LockedAndLoaded;
        } else if (hasNote) {
            if (blueAlliance) {
                pattern = LedsIO.ColorPattern.LoadedBlue;
            } else {
                pattern = LedsIO.ColorPattern.LoadedRed;
            }
        } else if (armOut && hasNote) {
            pattern = LedsIO.ColorPattern.Loaded;
        } else if (armOut) {
            pattern = LedsIO.ColorPattern.Loading;
        } else if (blueAlliance) {
            pattern = LedsIO.ColorPattern.DefaultBlue;
        } else {
            pattern = LedsIO.ColorPattern.DefaultRed;
        }

        if (currentPattern != pattern) {
            currentPattern = pattern;
            io.setColorPattern(pattern);
        }
    }

    @AutoLogOutput
    public LedsIO.ColorPattern getCurrentColorPattern() {
        return currentPattern;
    }

}
