package frc.robot.subsystems.intake;

import frc.utils.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Intake/IntakeVoltage", IntakeConstants.kIntakeVoltage);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void ingest() {
    ingest(intakeVoltage.getAsDouble());
  }

  public void ingest(double voltage) {
    io.ingest(voltage);
  }

  public void expel(double voltage) {
    io.expel(voltage);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public boolean hasNote() {
    return inputs.hasNote;
  }

  @AutoLogOutput
  public boolean isIntakeActive(){
    return !(inputs.appliedVoltsLeft == 0 && inputs.appliedVoltsRight == 0);
  }
  
}
