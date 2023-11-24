package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Hand;
import com.team1701.frc2023.subsystems.Hand.HandState;

public class HandTask implements Task {
    private final HandState mDesiredHandState;
    private final Hand mHand = Hand.getInstance();
    private final boolean mWaitForActuation;

    public HandTask(HandState desiredState) {
        this(desiredState, false);
    }

    public HandTask(HandState desiredState, boolean waitForActuation) {
        mDesiredHandState = desiredState;
        mWaitForActuation = waitForActuation;
    }

    @Override
    public void start() {
        mHand.setHandState(mDesiredHandState);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return !mWaitForActuation
                || (mDesiredHandState == HandState.OPEN && mHand.isOpen())
                || (mDesiredHandState == HandState.CLOSE && mHand.isClosed());
    }
}
