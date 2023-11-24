package com.team1701.frc2023.loops;

import java.util.ArrayList;
import java.util.List;

import com.team1701.lib.util.CrashTracker;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    private final List<Loop> mLoops;
    private double mTimestamp = 0;
    private double mDT = 0;
    private boolean mRunning;
    private String mName;

    public Looper(String name) {
        mRunning = false;
        mLoops = new ArrayList<>();
        mName = name;
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    public void start() {
        if (!mRunning) {
            System.out.println("Starting " + mName + " loops");
            mTimestamp = Timer.getFPGATimestamp();
            for (Loop loop : mLoops) {
                loop.onStart(mTimestamp);
            }
            mRunning = true;
        }
    }

    public void loop() {
        try {
            if (mRunning) {
                double now = Timer.getFPGATimestamp();

                for (Loop loop : mLoops) {
                    loop.onLoop(now);
                }

                mDT = now - mTimestamp;
                mTimestamp = now;
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void stop() {
        if (mRunning) {
            System.out.println("Stopping " + mName + " loops");
            mRunning = false;
            mTimestamp = Timer.getFPGATimestamp();
            for (Loop loop : mLoops) {
                System.out.println("Stopping " + loop);
                loop.onStop(mTimestamp);
            }
        }
    }

    public void outputToSmartDashboard() {
        var logger = Logger.getInstance();
        logger.recordOutput("Looper/" + mName + "/Running", mRunning);
        logger.recordOutput("Looper/" + mName + "/Dt", mDT);
        logger.recordOutput("Looper/" + mName + "/Timestamp", mTimestamp);
    }
}
