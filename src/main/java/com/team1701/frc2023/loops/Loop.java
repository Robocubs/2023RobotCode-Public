package com.team1701.frc2023.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {
    void onStart(double timestamp);

    void onLoop(double timestamp);

    void onStop(double timestamp);

    String getDisplayName();
}
