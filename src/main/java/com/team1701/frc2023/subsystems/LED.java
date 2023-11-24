package com.team1701.frc2023.subsystems;

import com.team1701.lib.drivers.LEDController;
import com.team1701.lib.drivers.LEDController.LightColor;
import com.team1701.lib.util.LEDState;
import edu.wpi.first.wpilibj.I2C;

public class LED extends Subsystem {
    private static LED mInstance;
    private LEDController mLedController;
    private LightColor mDriveTeamColor = LEDState.kRobotStateDisabled;
    private LightColor mHumanPlayerColor = LEDState.kRobotStateDisabled;
    private LightColor mLastFrontColor = LEDState.kNone;
    private LightColor mLastBackColor = LEDState.kNone;
    private boolean mReversed = false;

    public static synchronized LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mLedController = new LEDController(I2C.Port.kMXP, 4);
    }

    public LightColor getDriveTeamColor() {
        return mDriveTeamColor;
    }

    public void setDriveTeamColor(LightColor color) {
        mDriveTeamColor = color;
    }

    public void setHumanPlayerColor(LightColor color) {
        mHumanPlayerColor = color;
    }

    public void setReversed(boolean reversed) {
        mReversed = reversed;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public void pushStateToLEDControllers() {
        LightColor frontColor;
        LightColor backColor;
        if (mReversed) {
            frontColor = mHumanPlayerColor;
            backColor = mDriveTeamColor;
        } else {
            frontColor = mDriveTeamColor;
            backColor = mHumanPlayerColor;
        }

        if (frontColor != mLastFrontColor || backColor != mLastBackColor) {
            mLedController.sendColor(frontColor, backColor);
            mLastFrontColor = frontColor;
            mLastBackColor = backColor;
        }
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
