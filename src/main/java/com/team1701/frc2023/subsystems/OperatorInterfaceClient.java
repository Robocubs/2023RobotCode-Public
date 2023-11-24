package com.team1701.frc2023.subsystems;

import com.team1701.frc2023.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

public class OperatorInterfaceClient extends Subsystem {
    private static OperatorInterfaceClient mInstance = null;
    private boolean mOperatorServerInited = false;
    private LoggedDashboardBoolean mOperatorServerInitedLogged =
            new LoggedDashboardBoolean("operatorserverloaded", false);
    public Integer mRowNumber = 0;
    public Integer mColumnNumber = 0;
    private INTFACTION mMainAction = INTFACTION.NONE;
    private Superstructure mSuperstructure;

    public static OperatorInterfaceClient getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterfaceClient();
        }
        return mInstance;
    }

    public OperatorInterfaceClient() {
        mSuperstructure = Superstructure.getInstance();
    }

    public enum INTFACTION {
        NONE,
        INVALID,
        SUBLEFT,
        SUBRIGHT,
        POSITION,
        PIECE,
        INTAKEEXTEND,
        INTAKERETRACT,
        INTAKEREJECT,
        INTAKETOGGLE,
        CLAWTOGGLE,
        ARMHOME,
        DUNK,
        COMPRESS
    }

    private void decodeActionID(String id) {
        if (Constants.kActionMap.containsKey(id)) {
            mMainAction = Constants.kActionMap.get(id);
        } else if (id.contains("r")) {
            mMainAction = INTFACTION.PIECE;
            var rowDelimeterIndex = id.indexOf("r");
            mRowNumber = (Integer) Character.getNumericValue(id.charAt(rowDelimeterIndex + 1));
            mColumnNumber = (Integer) Character.getNumericValue(id.charAt(rowDelimeterIndex + 2));
        } else {
            System.out.println("Received bad id: " + id);
            mMainAction = INTFACTION.INVALID;
        }
    }

    @Override
    public void readPeriodicInputs() {
        var actionRequested = new LoggedDashboardBoolean("actionRequested", false);
        var requestedAction = new LoggedDashboardString("requestedAction", "00");
        // Once the server inits, we can stop polling
        if (!mOperatorServerInited) {
            mOperatorServerInited = mOperatorServerInitedLogged.get();
            mMainAction = INTFACTION.NONE;
            return;
        }
        if (actionRequested.get()) {
            actionRequested.set(false);
            decodeActionID(requestedAction.get());
            mSuperstructure.requestOperatorAction(mMainAction);
        } else {
            mMainAction = INTFACTION.NONE;
        }
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("OperatorServer/Initialized", mOperatorServerInited);
    }
}
