package frc.robot.subsystems.leds;

import java.util.Map;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LedsIOBlinkin implements LedsIO {

    public static final int kBlinkinPWMId = 0;

    public static final double cDarkGray = 0.97;
    public static final double cBreathRed = -0.17;
    public static final double cBreathBlue = -0.15;
    public static final double cStrobeRed = -0.11;
    public static final double cStrobeBlue = -0.09;
    public static final double cStrobeGold = -0.07;
    public static final double cSolidGold = 0.67;
    public static final double cRainbow = -0.99;

    public static final Map<LedsIO.ColorPattern, Double> BlinkinColorMap = Map.of(
            LedsIO.ColorPattern.DefaultRed, cBreathRed,
            LedsIO.ColorPattern.DefaultBlue, cBreathBlue,
            LedsIO.ColorPattern.LoadedRed, cStrobeRed,
            LedsIO.ColorPattern.LoadedBlue, cStrobeBlue,
            LedsIO.ColorPattern.Loading, cStrobeGold,
            LedsIO.ColorPattern.Loaded, cSolidGold,
            LedsIO.ColorPattern.LockedAndLoaded, cRainbow);

    /*
     * Rev Robotics Blinkin takes a PWM signal from 1000-2000us
     * This is identical to a SparkMax motor.
     * -1 corresponds to 1000us
     * 0 corresponds to 1500us
     * +1 corresponds to 2000us
     */
    private static Spark blinkin = null;

    public LedsIOBlinkin() {
        blinkin = new Spark(kBlinkinPWMId);
    }

    public void setColorPattern(ColorPattern pattern) {
        blinkin.set(pattern == null ? cDarkGray : BlinkinColorMap.get(pattern));
    }

}
