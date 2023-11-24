package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Drive;
import com.team1701.frc2023.subsystems.Drive.DriveControlState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TimedDriveTask extends TimedTask {

    private final ChassisSpeeds mSpeed;

    public TimedDriveTask(ChassisSpeeds s, double duration) {
        super(duration);
        mSpeed = s;
    }

    @Override
    public void start(double timestamp) {
        var drive = Drive.getInstance();
        drive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        drive.setVelocity(mSpeed);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {
        Drive.getInstance().setVelocity(new ChassisSpeeds(0, 0, 0));
    }
}
