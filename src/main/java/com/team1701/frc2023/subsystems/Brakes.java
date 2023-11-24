package com.team1701.frc2023.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.lib.drivers.CubDoubleSolenoid;
import com.team1701.lib.util.TimeLockedBoolean;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Brakes extends Subsystem {
    private static Brakes mInstance = null;

    // See Brake class definition below
    private final Brake mRotationBrake;
    private final Brake mExtensionBrake;
    private final List<Brake> mBrakeList;

    public static Brakes getInstance() {
        if (mInstance == null) {
            mInstance = new Brakes();
        }
        return mInstance;
    }

    private Brakes() {
        mRotationBrake = new Brake(
                "RotationBrake", Constants.kArmRotationBrakeEngageId, Constants.kArmRotationBrakeDisengageId, 1.0);
        mExtensionBrake = new Brake(
                "ExtensionBrake", Constants.kArmExtensionBrakeEngageId, Constants.kArmExtensionBrakeDisengageId, 1.0);
        mBrakeList = new ArrayList<Brake>(2);
        mBrakeList.add(mRotationBrake); // 0
        mBrakeList.add(mExtensionBrake); // 1
    }

    public void engageRotationBrake() {
        mRotationBrake.engageBrake();
    }

    public void disengageRotationBrake() {
        mRotationBrake.disengageBrake();
    }

    public void toggleRotationBrake() {
        mRotationBrake.toggleBrake();
    }

    public void engageExtensionBrake() {
        mExtensionBrake.engageBrake();
    }

    public void disengageExtensionBrake() {
        mExtensionBrake.disengageBrake();
    }

    public void toggleExtensionBrake() {
        mExtensionBrake.toggleBrake();
    }

    public BrakeState getRotationBrakeState() {
        return mRotationBrake.mBrakeState;
    }

    public BrakeState getExtensionBrakeState() {
        return mExtensionBrake.mBrakeState;
    }

    public boolean rotationBrakeFinishedEngaging() {
        return mRotationBrake.mFinishedEngaging.getValue();
    }

    public boolean rotationBrakeFinishedDisengaging() {
        return mRotationBrake.mFinishedDisengaging.getValue();
    }

    public boolean extensionBrakeFinishedEngaging() {
        return mExtensionBrake.mFinishedEngaging.getValue();
    }

    public boolean extensionBrakeFinishedDisengaging() {
        return mExtensionBrake.mFinishedDisengaging.getValue();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mBrakeList.forEach(Brake::handleStartup);
            }

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Brakes.class.getSimpleName();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {}

    @Override
    public void writePeriodicOutputs() {
        mBrakeList.forEach(Brake::updateSolenoidDemand);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mBrakeList.forEach(Brake::outputState);
    }

    public static enum BrakeState {
        ENGAGED,
        DISENGAGED
    }

    private static class Brake {
        private final CubDoubleSolenoid mBackingSolenoid;
        private final String mBrakeName;
        private final double mMinimumSecondsBetweenEngaging;
        private final TimeLockedBoolean mFinishedEngaging =
                new TimeLockedBoolean(Constants.kBrakeActuationTime, Timer.getFPGATimestamp());
        private final TimeLockedBoolean mFinishedDisengaging =
                new TimeLockedBoolean(Constants.kBrakeActuationTime, Timer.getFPGATimestamp());

        private DoubleSolenoid.Value mSolenoidDemand = DoubleSolenoid.Value.kOff;
        private BrakeState mBrakeState = BrakeState.ENGAGED;
        private double mLastDisengageTimestamp = 0;

        public Brake(String brakeName, int engageId, int disengageId, double minimumSecondsBetweenEngaging) {
            mBrakeName = brakeName;
            mBackingSolenoid = new CubDoubleSolenoid(
                    Constants.kPneumaticsHubId, PneumaticsModuleType.REVPH, engageId, disengageId);
            mMinimumSecondsBetweenEngaging = minimumSecondsBetweenEngaging;
        }

        public void toggleBrake() {
            if (mBrakeState == BrakeState.ENGAGED) {
                disengageBrake();
            } else {
                engageBrake();
            }
        }

        public void engageBrake() {
            mBrakeState = BrakeState.ENGAGED;
        }

        public void disengageBrake() {
            mBrakeState = BrakeState.DISENGAGED;
        }

        public void updateSolenoidDemand() {
            var timestamp = Timer.getFPGATimestamp();
            if (mSolenoidDemand == DoubleSolenoid.Value.kForward && mBrakeState == BrakeState.DISENGAGED) {
                mLastDisengageTimestamp = timestamp;
            }

            if (mBrakeState == BrakeState.ENGAGED
                    && timestamp > mLastDisengageTimestamp + mMinimumSecondsBetweenEngaging) {
                mSolenoidDemand = DoubleSolenoid.Value.kForward;
            } else {
                mSolenoidDemand = DoubleSolenoid.Value.kReverse;
            }

            mBackingSolenoid.set(mSolenoidDemand);

            mFinishedEngaging.update(mSolenoidDemand == DoubleSolenoid.Value.kForward, timestamp);
            mFinishedDisengaging.update(mSolenoidDemand == DoubleSolenoid.Value.kReverse, timestamp);
        }

        public void handleStartup() {
            if (mBackingSolenoid.get() == DoubleSolenoid.Value.kReverse) {
                mBrakeState = BrakeState.DISENGAGED;
            } else {
                mBrakeState = BrakeState.ENGAGED;
            }

            mBackingSolenoid.clearCache();
        }

        public void outputState() {
            Logger.getInstance().recordOutput("Brake/" + mBrakeName + "/State", mBrakeState.toString());
        }
    }
}
