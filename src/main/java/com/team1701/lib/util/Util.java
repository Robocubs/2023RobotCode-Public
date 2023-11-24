package com.team1701.lib.util;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static boolean inRangeInclusive(double v, double maxMagnitude) {
        return inRangeInclusive(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRangeInclusive(double v, double min, double max) {
        return v >= min && v <= max;
    }
}
