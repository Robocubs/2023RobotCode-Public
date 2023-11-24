package com.team1701.lib.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

// A non-command version of PPSwerveControllerCommand
// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/commands/PPSwerveControllerCommand.java
public class SwerveTrajectoryController {
    private final Timer mTimer = new Timer();
    private final PPHolonomicDriveController mHolonomicDriveController;
    private PathPlannerTrajectory mTrajectory = null;
    private Pose2d mDesiredPose = GeometryUtil.kPoseIdentity;
    private Pose2d mError = GeometryUtil.kPoseIdentity;

    public SwerveTrajectoryController(
            PIDController xController, PIDController yController, PIDController rotationController) {
        mHolonomicDriveController = new PPHolonomicDriveController(xController, yController, rotationController);
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        mTrajectory = trajectory;

        mTimer.reset();
        mTimer.start();

        PathPlannerServer.sendActivePath(trajectory.getStates());
    }

    public ChassisSpeeds update(Pose2d currentPose) {
        var currentTime = mTimer.get();
        var desiredState = (PathPlannerState) mTrajectory.sample(currentTime);

        mDesiredPose = new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation);
        mError = mDesiredPose.relativeTo(currentPose);

        PathPlannerServer.sendPathFollowingData(mDesiredPose, currentPose);

        return mHolonomicDriveController.calculate(currentPose, desiredState);
    }

    public boolean isFinished() {
        return mTrajectory != null && mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds());
    }

    public Pose2d getDesiredPose() {
        return mDesiredPose;
    }

    public Pose2d getError() {
        return mError;
    }
}
