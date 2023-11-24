package com.team1701.frc2023.subsystems;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.lib.drivers.CubDoubleSolenoid;
import com.team1701.lib.drivers.RetroreflectiveSensor;
import com.team1701.lib.util.TimeLockedBoolean;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Hand extends Subsystem {
    private static Hand mInstance;
    private HandState mHandState = HandState.CLOSE;
    private final HandOutputs mPeriodicIO;
    private final CubDoubleSolenoid mHandActuator;
    private final RetroreflectiveSensor mHandSensor;
    private boolean mSensesObject;
    private final TimeLockedBoolean mIsOpen =
            new TimeLockedBoolean(Constants.kHandActuationTime, Timer.getFPGATimestamp());
    private final TimeLockedBoolean mIsClosed =
            new TimeLockedBoolean(Constants.kHandActuationTime, Timer.getFPGATimestamp());
    private HandInputsAutoLogged mHandInputs = new HandInputsAutoLogged();

    public static Hand getInstance() {
        if (mInstance == null) {
            mInstance = new Hand();
        }
        return mInstance;
    }

    public enum HandState {
        OPEN,
        CLOSE,
        AUTO_CLOSE,
        NONE;
    }

    @AutoLog
    public static class HandInputs {
        public boolean sensesObject;
    }

    private static class HandOutputs {
        public DoubleSolenoid.Value solenoidDemand = DoubleSolenoid.Value.kOff;
    }

    public Hand() {
        mPeriodicIO = new HandOutputs();
        mPeriodicIO.solenoidDemand = DoubleSolenoid.Value.kReverse;

        mHandActuator = new CubDoubleSolenoid(
                Constants.kPneumaticsHubId,
                PneumaticsModuleType.REVPH,
                Constants.kHandCloseId,
                Constants.kHandReleaseId);
        mHandSensor = new RetroreflectiveSensor(Constants.kHandSensorID);
        mHandSensor.setReversed(true);
    }

    public void setHandState(HandState state) {
        mHandState = state;
    }

    public HandState getHandState() {
        return mHandState;
    }

    public boolean isOpen() {
        return mIsOpen.getValue();
    }

    public boolean isClosed() {
        return mIsClosed.getValue();
    }

    public boolean seesObject() {
        return mSensesObject;
    }

    public void toggleHandState() {
        mHandState = mHandState == HandState.CLOSE ? HandState.OPEN : HandState.CLOSE;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mHandActuator.clearCache();
            }

            @Override
            public void onLoop(double timestamp) {
                if (mHandState == HandState.AUTO_CLOSE) {
                    // Do not auto open, just auto-close
                    if (mSensesObject) {
                        mPeriodicIO.solenoidDemand = DoubleSolenoid.Value.kReverse;
                        mHandState = HandState.CLOSE;
                    }
                } else {
                    mPeriodicIO.solenoidDemand = mHandState == HandState.OPEN
                            ? DoubleSolenoid.Value.kForward
                            : DoubleSolenoid.Value.kReverse;
                }

                mIsOpen.update(mPeriodicIO.solenoidDemand == DoubleSolenoid.Value.kForward, timestamp);
                mIsClosed.update(mPeriodicIO.solenoidDemand == DoubleSolenoid.Value.kReverse, timestamp);
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Hand.class.getSimpleName();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mSensesObject = mHandSensor.get();

        Logger.getInstance().processInputs("Hand", mHandInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mHandActuator.set(mPeriodicIO.solenoidDemand);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("Hand/SensesObject", mSensesObject);
        Logger.getInstance().recordOutput("Hand/State", mHandState.toString());
    }
}
