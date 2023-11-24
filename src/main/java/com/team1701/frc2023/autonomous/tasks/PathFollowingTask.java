package com.team1701.frc2023.autonomous.tasks;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.team1701.frc2023.subsystems.Drive;
import com.team1701.frc2023.subsystems.Drive.DriveControlState;
import com.team1701.frc2023.subsystems.Intake;
import com.team1701.frc2023.subsystems.PieceSeeker;
import com.team1701.lib.util.Callback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class PathFollowingTask implements Task {

    private final Drive mDrive = Drive.getInstance();
    private final Supplier<PathPlannerTrajectory> mTrajectorySupplier;
    private PathPlannerTrajectory mTrajectory;
    private boolean mRunning = false;
    private Timer mTimer = new Timer();
    private List<EventMarker> mUnpassedMarkers = List.of();
    private Map<String, Callback> mEventCallbacks;
    private boolean mPieceSeekingEnabled = false;
    private boolean mShouldWaitForPieceWaypointCompletion = false;
    private PieceSeeker mPieceSeeker = PieceSeeker.getInstance();

    public PathFollowingTask(Supplier<PathPlannerTrajectory> trajectorySupplier) {
        this.mTrajectorySupplier = trajectorySupplier;
        this.mEventCallbacks = Map.of();
    }

    public PathFollowingTask(Supplier<PathPlannerTrajectory> trajectorySupplier, Map<String, Callback> map) {
        this.mTrajectorySupplier = trajectorySupplier;
        this.mEventCallbacks = map;
    }

    public PathFollowingTask(
            Supplier<PathPlannerTrajectory> trajectorySupplier, Map<String, Callback> map, boolean PSEnabled) {
        this.mTrajectorySupplier = trajectorySupplier;
        this.mEventCallbacks = map;
        this.mPieceSeekingEnabled = PSEnabled;
    }

    @Override
    public void start() {
        if (mRunning) {
            return;
        }
        mShouldWaitForPieceWaypointCompletion = false;
        mTrajectory = mTrajectorySupplier.get();
        mUnpassedMarkers = new ArrayList<>(mTrajectory.getMarkers());

        mDrive.setPath(mTrajectory);
        mPieceSeeker.setExpectedPosition(mTrajectory.getEndState().poseMeters.getTranslation());

        mTimer.reset();
        mTimer.start();
        mRunning = true;
    }

    @Override
    public void update() {
        // There's better ways to write this but I'm tired
        if (mPieceSeekingEnabled) {
            Optional<Pose2d> pos = mPieceSeeker.getPiecePosition();
            if (!pos.isEmpty()) {
                if (Intake.getInstance().hasPiece()) {
                    mPieceSeekingEnabled = false;
                    mDrive.setDriveControlState(DriveControlState.PATH_FOLLOWING);
                    return;
                }
                DriverStation.reportWarning("CORRECTING AUTON", false);
                mDrive.setWaypoint(pos.get());
                mShouldWaitForPieceWaypointCompletion = true;
            }
        }

        if (mUnpassedMarkers.isEmpty()) return;

        // Has our robot's timer passed the marker's execution time
        if (mTimer.get() >= mUnpassedMarkers.get(0).timeSeconds) {
            // If so, we remove the marker from the unpassed markers list
            EventMarker marker = mUnpassedMarkers.remove(0);
            // Then iterate through the names of the singular marker
            for (String name : marker.names) {
                // If any matches
                if (mEventCallbacks.containsKey(name)) {
                    // Then we execute the callbacks
                    mEventCallbacks.get(name).invoke();
                }
            }
        }
    }

    @Override
    public void stop() {
        mDrive.setVelocity(new ChassisSpeeds(0, 0, 0));
        mRunning = false;
    }

    @Override
    public boolean isCompleted() {
        var canFinishPath =
                !mShouldWaitForPieceWaypointCompletion || Intake.getInstance().hasPiece() || mDrive.robotIsAtWaypoint();
        return mTrajectory == null || (mTimer.get() > mTrajectory.getTotalTimeSeconds() && canFinishPath);
    }
}
