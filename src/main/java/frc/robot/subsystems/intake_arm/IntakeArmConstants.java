package frc.robot.subsystems.intake_arm;

public class IntakeArmConstants {
    public static final Gains gains = new Gains(0.7, 0.8, 0.05);

    public static final double ff = 0.1;

    public static final double loadPosition = 0.345;
    public static final double intakePosition = 0.97;
    public static final double ampPosition = 0.715;

    public static final double outThreshold = 0.55;

    public record Gains(double kP, double kI, double kD) {}
}
