package frc.robot.subsystems.leds;

public interface LedsIO {
    enum ColorPattern {
        DefaultRed,
        DefaultBlue,
        LoadedRed,
        LoadedBlue,
        Loading,
        Loaded,
        LockedAndLoaded,
    }

    default void setColorPattern(ColorPattern pattern) {}
}
