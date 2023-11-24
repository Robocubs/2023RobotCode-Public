package com.team1701.lib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveWaypointController {
    private final PIDController mPositionController;
    private final PIDController mRotationController;
    private final TrapezoidProfile.Constraints mConstraints;
    private final double mDt;
    Pose2d mGoal = new Pose2d();
    Translation2d mSetpoint = new Translation2d();
    private double mDistanceToGoal;

    public SwerveWaypointController(
            PIDController positionController,
            PIDController rotationController,
            TrapezoidProfile.Constraints constraints,
            double dt,
            double positionTolerance,
            Rotation2d rotationTolerance) {
        mPositionController = positionController;
        mRotationController = rotationController;
        mConstraints = constraints;
        mDt = dt;

        mPositionController.setTolerance(positionTolerance);
        mRotationController.setTolerance(rotationTolerance.getRadians());
        mRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setGoal(Pose2d goal, Pose2d currentPose) {
        mGoal = goal;
        mSetpoint = currentPose.getTranslation();
        mDistanceToGoal = mGoal.getTranslation().getDistance(currentPose.getTranslation());
        mPositionController.reset();
        mRotationController.setSetpoint(goal.getRotation().getRadians());
        mRotationController.reset();
    }

    public Pose2d getGoal() {
        return mGoal;
    }

    public boolean atWaypoint() {
        return mDistanceToGoal < mPositionController.getPositionTolerance() && mRotationController.atSetpoint();
    }

    public ChassisSpeeds calculate(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        var translationToGoal = mGoal.getTranslation().minus(currentPose.getTranslation());
        mDistanceToGoal = translationToGoal.getNorm();

        var rotationPidOutput =
                mRotationController.calculate(currentPose.getRotation().getRadians());

        var positionError = mSetpoint.minus(currentPose.getTranslation());
        var positionPidOutput = positionError.times(mPositionController.calculate(0, positionError.getNorm()));

        if (translationToGoal.getNorm() < mPositionController.getPositionTolerance()) {
            mSetpoint = currentPose.getTranslation();
            return mRotationController.atSetpoint() ? new ChassisSpeeds() : new ChassisSpeeds(0, 0, rotationPidOutput);
        }

        var currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        var currentState = new TrapezoidProfile.State(0, currentVelocity);
        var goalState = new TrapezoidProfile.State(translationToGoal.getNorm(), 0);
        var newState = new TrapezoidProfile(mConstraints, goalState, currentState).calculate(mDt);
        var feedForwardVelocity = new Translation2d(newState.velocity, translationToGoal.getAngle());

        mSetpoint =
                currentPose.getTranslation().plus(new Translation2d(newState.position, translationToGoal.getAngle()));

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                feedForwardVelocity.getX() + positionPidOutput.getX(),
                feedForwardVelocity.getY() + positionPidOutput.getY(),
                rotationPidOutput,
                currentPose.getRotation());
    }
}
