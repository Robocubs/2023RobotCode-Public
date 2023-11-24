package com.team1701.frc2023.autonomous.tasks;

import com.team1701.frc2023.subsystems.Drive;
import edu.wpi.first.math.geometry.Pose2d;

public class WaypointTask implements Task {

    private Drive mDrive = Drive.getInstance();
    private final Pose2d mWaypointPose;
    private final boolean mWaitForWaypoint;

    public WaypointTask(Pose2d pose, boolean waitForWaypoint) {
        mWaypointPose = pose;
        mWaitForWaypoint = waitForWaypoint;
    }

    @Override
    public void start() {
        mDrive.setWaypoint(mWaypointPose);
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return !mWaitForWaypoint || mDrive.robotIsAtWaypoint();
    }
}
