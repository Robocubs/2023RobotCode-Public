package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Drive;
import com.team1701.frc2023.subsystems.Drive.DriveControlState;

public class AutoBalanceTask implements Task {

    @Override
    public void start() {
        Drive.getInstance().setDriveControlState(DriveControlState.AUTO_BALANCE);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return false;
    }
}
