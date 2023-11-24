package com.team1701.frc2023.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;

public abstract class TimedTask implements Task {

    private final double mMinDuration;
    private final double mMaxDuration;
    private double mStartTime = Integer.MAX_VALUE;

    public TimedTask(double duration) {
        this(0, duration);
    }

    public TimedTask(double minDuration, double maxDuration) {
        mMinDuration = minDuration;
        mMaxDuration = maxDuration;
    }

    @Override
    public final void start() {
        mStartTime = Timer.getFPGATimestamp();
        start(mStartTime);
    }

    protected abstract void start(double timestamp);

    @Override
    public final boolean isCompleted() {
        var getTimeSinceStarted = getTimeSinceStarted();
        return getTimeSinceStarted >= mMaxDuration || (getTimeSinceStarted >= mMinDuration && isCompletedEarly());
    }

    protected boolean isCompletedEarly() {
        return false;
    }

    protected double getDuration() {
        return mMaxDuration;
    }

    protected double getTimeSinceStarted() {
        return Timer.getFPGATimestamp() - mStartTime;
    }

    protected double getTimeRemaining() {
        return mStartTime + mMaxDuration - Timer.getFPGATimestamp();
    }
}
