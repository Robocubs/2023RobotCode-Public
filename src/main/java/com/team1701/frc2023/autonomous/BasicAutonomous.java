package com.team1701.frc2023.autonomous;

import org.littletonrobotics.junction.Logger;

public class BasicAutonomous {

    private BasicAutonomousState mState = BasicAutonomousState.NONE;

    public void reset(double timestamp) {
        mState = BasicAutonomousState.NONE;
    }

    public void loop(double timestamp) {
        switch (mState) {
            case NONE:
                break;
        }
        Logger.getInstance().recordOutput("Autonomous/State", mState.toString());
    }

    private enum BasicAutonomousState {
        NONE
    }
}
