package com.team1701.frc2023.autonomous.tasks;

public interface Task {

    public void start();

    public void update();

    public void stop();

    public boolean isCompleted();
}
