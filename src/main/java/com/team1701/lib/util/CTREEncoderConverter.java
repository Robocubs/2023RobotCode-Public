package com.team1701.lib.util;

import edu.wpi.first.math.util.Units;

public class CTREEncoderConverter {

    private final int mCountsPerRev;
    private final double mGearRatio;
    private final double mWheelCircumferenceMeters;

    public CTREEncoderConverter(int encoderCountsPerRev, double gearRatio, double wheelDiameterInches) {
        mCountsPerRev = encoderCountsPerRev;
        mGearRatio = gearRatio;
        mWheelCircumferenceMeters = Math.PI * Units.inchesToMeters(wheelDiameterInches);
    }

    public int distanceToNativeUnits(double positionMeters) {
        var wheelRotations = positionMeters / mWheelCircumferenceMeters;
        var motorRotations = wheelRotations * mGearRatio;
        var sensorCounts = (int) (motorRotations * mCountsPerRev);
        return sensorCounts;
    }

    public int velocityToNativeUnits(double velocityMetersPerSecond) {
        var wheelRotationsPerSecond = velocityMetersPerSecond / mWheelCircumferenceMeters;
        var motorRotationsPerSecond = wheelRotationsPerSecond * mGearRatio;
        var motorRotationsPer100ms = motorRotationsPerSecond / 10;
        var sensorCountsPer100ms = (int) (motorRotationsPer100ms * mCountsPerRev);
        return sensorCountsPer100ms;
    }

    public double nativeUnitsToDistanceMeters(double sensorCounts) {
        var motorRotations = sensorCounts / mCountsPerRev;
        var wheelRotations = motorRotations / mGearRatio;
        var positionMeters = wheelRotations * mWheelCircumferenceMeters;
        return positionMeters;
    }

    public double nativeUnitsToVelocityMetersPerSecond(double sensorCountsPer100ms) {
        var motorRotationsPer100ms = sensorCountsPer100ms / mCountsPerRev;
        var motorRotationsPerSecond = motorRotationsPer100ms * 10;
        var wheelRotationsPerSecond = motorRotationsPerSecond / mGearRatio;
        var velocityMetersPerSecond = wheelRotationsPerSecond * mWheelCircumferenceMeters;
        return velocityMetersPerSecond;
    }
}
