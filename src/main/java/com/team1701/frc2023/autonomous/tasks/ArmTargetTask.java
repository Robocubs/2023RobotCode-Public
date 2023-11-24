package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Arm;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;

public class ArmTargetTask implements Task {
    private final Arm mArm = Arm.getInstance();
    private TargetType mTargetType = null;
    private final boolean mWaitForPosition;
    private final boolean mPremove;

    public ArmTargetTask(TargetType type, boolean premove, boolean waitForPosition) {
        mWaitForPosition = waitForPosition;
        mTargetType = type;
        mPremove = premove;
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public void start() {
        Superstructure.getInstance().setTargetType(mTargetType, mPremove);
    }

    @Override
    public boolean isCompleted() {
        return !mWaitForPosition || mArm.atTarget();
    }
}
