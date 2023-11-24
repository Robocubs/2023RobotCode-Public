package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Intake;
import com.team1701.frc2023.subsystems.Intake.IntakeState;

public class IntakeTargetTask implements Task {

    private IntakeState mDesIntakeState;
    private boolean mWaitForPosition = false;
    private Intake mIntake;

    public IntakeTargetTask(IntakeState desState, boolean waitForPosition) {
        mIntake = Intake.getInstance();
        mDesIntakeState = desState;
        mWaitForPosition = waitForPosition;
    }

    @Override
    public void start() {
        mIntake.setIntakeState(mDesIntakeState);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return !mWaitForPosition || mIntake.atTarget();
    }
}
