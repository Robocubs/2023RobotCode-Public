package com.team1701.frc2023;

import java.lang.reflect.Method;
import java.util.PriorityQueue;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import org.littletonrobotics.junction.Logger;

/**
 * LoggedRobot implements the IterativeRobotBase robot program framework.
 *
 * <p>
 * The LoggedRobot class is intended to be subclassed by a user creating a robot
 * program, and will call all required AdvantageKit periodic methods.
 *
 * <p>
 * periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
public class LoggedRobot extends IterativeRobotBase {
    public static final double defaultPeriodSecs = 0.02;
    private final int notifier = NotifierJNI.initializeNotifier();
    private final long startTime;
    private final PriorityQueue<Callback> callbacks = new PriorityQueue<>();
    private final LoggerWrapper loggerWrapper = new LoggerWrapper();

    private boolean useTiming = true;

    /** Constructor for LoggedRobot. */
    protected LoggedRobot() {
        this(defaultPeriodSecs);
    }

    /**
     * Constructor for LoggedRobot.
     *
     * @param period Period in seconds.
     */
    protected LoggedRobot(double period) {
        super(period);
        startTime = Logger.getInstance().getRealTimestamp();
        addPeriodic(this::loopFunc, period);
        NotifierJNI.setNotifierName(notifier, "LoggedRobot");

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }

    @Override
    @SuppressWarnings("NoFinalizer")
    protected void finalize() {
        NotifierJNI.stopNotifier(notifier);
        NotifierJNI.cleanNotifier(notifier);
    }

    /** Provide an alternate "main loop" via startCompetition(). */
    @Override
    @SuppressWarnings("UnsafeFinalization")
    public void startCompetition() {
        robotInit();

        if (isSimulation()) {
            simulationInit();
        }

        // Save data from init cycle
        loggerWrapper.periodicAfterUser();

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            if (useTiming) {
                // We don't have to check there's an element in the queue first because
                // there's always at least one (the constructor adds one). It's reenqueued
                // at the end of the loop.
                var callback = callbacks.poll();

                NotifierJNI.updateNotifierAlarm(notifier, callback.expirationTime);

                var currentTime = NotifierJNI.waitForNotifierAlarm(notifier);
                if (currentTime == 0) {
                    break;
                }

                runCallbackWithLogging(callback);

                callback.expirationTime += callback.period;
                callbacks.add(callback);

                while (callbacks.peek().expirationTime <= currentTime) {
                    callback = callbacks.poll();

                    runCallbackWithLogging(callback);

                    callback.expirationTime += callback.period;
                    callbacks.add(callback);
                }
            } else {
                while (callbacks.peek().expirationTime <= Logger.getInstance().getTimestamp()) {
                    var callback = callbacks.poll();
                    runCallbackWithLogging(callback);
                    callback.expirationTime = callback.expirationTime += callback.period;
                    callbacks.add(callback);
                }
            }
        }
    }

    protected void runCallbackWithLogging(Callback callback) {
        long loopCycleStart = Logger.getInstance().getRealTimestamp();
        loggerWrapper.periodicBeforeUser();

        long userCodeStart = Logger.getInstance().getRealTimestamp();
        callback.func.run();
        long loopCycleEnd = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("LoggedRobot/FullCycleMS", (loopCycleEnd - loopCycleStart) / 1000.0);
        Logger.getInstance().recordOutput("LoggedRobot/LogPeriodicMS", (userCodeStart - loopCycleStart) / 1000.0);
        Logger.getInstance().recordOutput("LoggedRobot/UserCodeMS", (loopCycleEnd - userCodeStart) / 1000.0);

        loggerWrapper.periodicAfterUser();
    }

    /** Ends the main loop in startCompetition(). */
    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(notifier);
    }

    /** Sets whether to use standard timing or run as fast as possible. */
    public void setUseTiming(boolean useTiming) {
        this.useTiming = useTiming;
    }

    /**
     * Add a callback to run at a specific period.
     *
     * <p>This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     */
    public void addPeriodic(Runnable callback, double periodSeconds) {
        callbacks.add(new Callback(callback, startTime, periodSeconds, 0.0));
    }

    /**
     * Add a callback to run at a specific period with a starting time offset.
     *
     * <p>This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     * @param offsetSeconds The offset from the common starting time in seconds. This is useful for
     *     scheduling a callback in a different timeslot relative to TimedRobot.
     */
    public void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
        callbacks.add(new Callback(callback, startTime, periodSeconds, offsetSeconds));
    }

    private static class Callback implements Comparable<Callback> {
        public Runnable func;
        public long period;
        public long expirationTime;

        /**
         * Construct a callback container.
         *
         * @param func The callback to run.
         * @param startTimeMicroseconds The common starting point for all callback scheduling in microseconds.
         * @param periodSeconds The period at which to run the callback in seconds.
         * @param offsetSeconds The offset from the common starting time in seconds.
         */
        Callback(Runnable func, long startTimeMicroseconds, double periodSeconds, double offsetSeconds) {
            var offsetMicroseconds = (long) (offsetSeconds * 1e6);
            this.func = func;
            this.period = (long) (periodSeconds * 1e6);
            this.expirationTime = startTimeMicroseconds
                    + offsetMicroseconds
                    + (Logger.getInstance().getRealTimestamp() - startTimeMicroseconds)
                    + this.period;
        }

        @Override
        public boolean equals(Object rhs) {
            if (rhs instanceof Callback) {
                return Long.compare(expirationTime, ((Callback) rhs).expirationTime) == 0;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Long.hashCode(expirationTime);
        }

        @Override
        public int compareTo(Callback rhs) {
            // Elements with sooner expiration times are sorted as lesser. The head of
            // Java's PriorityQueue is the least element.
            return Long.compare(expirationTime, rhs.expirationTime);
        }
    }

    /**
     * Hacky approach to accessing package-level methods.
     */
    private static class LoggerWrapper {
        private Method periodicBeforeUser;
        private Method periodicAfterUser;

        private LoggerWrapper() {
            Method periodicBeforeUser = null;
            Method periodicAfterUser = null;
            try {
                periodicBeforeUser = Logger.class.getDeclaredMethod("periodicBeforeUser");
                periodicBeforeUser.setAccessible(true);
                periodicAfterUser = Logger.class.getDeclaredMethod("periodicAfterUser");
                periodicAfterUser.setAccessible(true);
            } catch (Exception ex) {
            }

            this.periodicBeforeUser = periodicBeforeUser;
            this.periodicAfterUser = periodicAfterUser;
        }

        private void periodicBeforeUser() {
            if (periodicBeforeUser != null) {
                try {
                    periodicBeforeUser.invoke(Logger.getInstance());
                } catch (Exception ex) {
                }
            }
        }

        private void periodicAfterUser() {
            if (periodicAfterUser != null) {
                try {
                    periodicAfterUser.invoke(Logger.getInstance());
                } catch (Exception ex) {
                }
            }
        }
    }
}
