package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Arm;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;

public class TimedArmTargetTask extends TimedTask {
    private final TargetType mTargetType;
    private final Arm mArm = Arm.getInstance();
    private final boolean mPremove;

    public TimedArmTargetTask(TargetType type, boolean premove, double mintime, double maxtime) {
        super(mintime, maxtime);
        mTargetType = type;
        mPremove = premove;
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    protected void start(double timestamp) {
        Superstructure.getInstance().setTargetType(mTargetType, mPremove);
    }

    @Override
    protected boolean isCompletedEarly() {
        return mArm.atTarget();
    }
}
