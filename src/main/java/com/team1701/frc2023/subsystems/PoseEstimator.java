package com.team1701.frc2023.subsystems;

import java.util.ConcurrentModificationException;

import com.team1701.frc2023.Constants;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends Subsystem {
    private static PoseEstimator mInstance = null;
    private SwerveDrivePoseEstimator poseEstimator;
    private Drive mDrive;
    private Pose2d mPose;
    private Field2d mField = new Field2d();

    public static PoseEstimator getInstance() {
        if (mInstance == null) {
            mInstance = new PoseEstimator();
        }
        return mInstance;
    }

    public PoseEstimator() {
        mPose = new Pose2d();
        mDrive = Drive.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.kKinematics,
                mDrive.getFieldRelativeGyroscopeRotation(),
                mDrive.getCurrentSwerveModulePositions(),
                new Pose2d());
    }

    public Pose2d getCurrentPose() {
        return mPose;
    }

    public void setPose(Pose2d pose) {
        try {
            poseEstimator.resetPosition(
                    mDrive.getFieldRelativeGyroscopeRotation(), mDrive.getCurrentSwerveModulePositions(), pose);
        } catch (ConcurrentModificationException e) {
            return;
        }
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    @Override
    public void readPeriodicInputs() {
        poseEstimator.update(mDrive.getFieldRelativeGyroscopeRotation(), mDrive.getCurrentSwerveModulePositions());
        mPose = poseEstimator.getEstimatedPosition();
        mField.setRobotPose(mPose);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("PoseEstimator/Pose", mPose);
    }
}
