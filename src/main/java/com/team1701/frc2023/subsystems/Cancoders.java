package com.team1701.frc2023.subsystems;

import java.util.Optional;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.team1701.frc2023.Constants;
import com.team1701.lib.drivers.TalonUtil;
import edu.wpi.first.wpilibj.Timer;

// Conatiner to hold the Cancoders so we can initialize them
// earlier than everything else and DI them to the swerve modules
public class Cancoders extends Subsystem {
    private final CANCoder mFrontLeft;
    private final CANCoder mFrontRight;
    private final CANCoder mBackLeft;
    private final CANCoder mBackRight;

    private final CanTsObserver mFrontRightObserver;
    private final CanTsObserver mFrontLeftObserver;
    private final CanTsObserver mBackLeftObserver;
    private final CanTsObserver mBackRightObserver;

    private static final double kBootUpErrorAllowanceTime = 10.0;

    private static class CanTsObserver {
        private final CANCoder cancoder;
        private Optional<Double> lastTs = Optional.empty();
        private int validUpdates = 0;
        private static final int kRequiredValidTimestamps = 10;

        public CanTsObserver(CANCoder cancoder) {
            this.cancoder = cancoder;
        }

        public boolean hasUpdate() {
            cancoder.getAbsolutePosition(); // Need to call this to update ts
            double ts = cancoder.getLastTimestamp();
            if (lastTs.isEmpty()) {
                lastTs = Optional.of(ts);
            }
            if (ts > lastTs.get()) {
                validUpdates++;
                lastTs = Optional.of(ts);
            }
            return validUpdates > kRequiredValidTimestamps;
        }
    }

    private static Cancoders sInstance;

    public static Cancoders getInstance() {
        if (sInstance == null) {
            sInstance = new Cancoders();
        }
        return sInstance;
    }

    private CANCoder build(int i) {
        CANCoder thisCancoder = new CANCoder(i);
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.magnetOffsetDegrees = 0.0;
        canCoderConfig.sensorDirection = false; // Counter-clockwise

        double startTime = Timer.getFPGATimestamp();
        boolean timedOut = false;
        boolean goodInit = false;
        int attempt = 1;
        while (!goodInit && !timedOut) {
            System.out.println("Initing CANCoder " + i + " / attempt: " + attempt);
            ErrorCode settingsError = thisCancoder.configAllSettings(canCoderConfig, Constants.kLongCANTimeoutMs);
            ErrorCode sensorDataError =
                    thisCancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50, Constants.kLongCANTimeoutMs);
            TalonUtil.checkError(settingsError, "Failed to configure CANCoder");
            TalonUtil.checkError(sensorDataError, "Failed to configure CANCoder update rate");

            goodInit = settingsError == ErrorCode.OK && sensorDataError == ErrorCode.OK;
            timedOut = (Timer.getFPGATimestamp()) - startTime >= kBootUpErrorAllowanceTime;
            attempt++;
        }

        return thisCancoder;
    }

    private Cancoders() {
        mFrontLeft = build(Constants.kFrontLeftEncoderPortId);
        mFrontLeftObserver = new CanTsObserver(mFrontLeft);

        mFrontRight = build(Constants.kFrontRightEncoderPortId);
        mFrontRightObserver = new CanTsObserver(mFrontRight);

        mBackLeft = build(Constants.kBackLeftEncoderPortId);
        mBackLeftObserver = new CanTsObserver(mBackLeft);

        mBackRight = build(Constants.kBackRightEncoderPortId);
        mBackRightObserver = new CanTsObserver(mBackRight);
    }

    public boolean allHaveBeenInitialized() {
        return mFrontLeftObserver.hasUpdate()
                && mFrontRightObserver.hasUpdate()
                && mBackLeftObserver.hasUpdate()
                && mBackRightObserver.hasUpdate();
    }

    public CANCoder getFrontLeft() {
        return mFrontLeft;
    }

    public CANCoder getFrontRight() {
        return mFrontRight;
    }

    public CANCoder getBackLeft() {
        return mBackLeft;
    }

    public CANCoder getBackRight() {
        return mBackRight;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}
}
