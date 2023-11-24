package com.team1701.frc2023.autonomous;

public interface AutonomousMode {

    public void start();

    public void loop();

    public void stop();

    public boolean isRunning();
}
