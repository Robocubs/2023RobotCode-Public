package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Drive;
import edu.wpi.first.math.geometry.Pose2d;

public class TimedWaypointTask extends TimedTask {
    private Drive mDrive = Drive.getInstance();
    private final Pose2d mWaypointPose;

    public TimedWaypointTask(Pose2d pose, double maxDuration) {
        super(0, maxDuration);
        mWaypointPose = pose;
    }

    @Override
    protected void start(double timestamp) {
        mDrive.setWaypoint(mWaypointPose);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompletedEarly() {
        return mDrive.robotIsAtWaypoint();
    }
}
