package com.team1701.frc2023.autonomous.tasks;

public abstract class SimpleTask implements Task {

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return true;
    }
}
