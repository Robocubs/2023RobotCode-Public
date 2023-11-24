package com.team1701.lib.trajectory;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathPlannerTransformer {
    private final double mFieldWidthMeters;

    public PathPlannerTransformer(double fieldWidthMeters) {
        mFieldWidthMeters = fieldWidthMeters;
    }

    // https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/PathPlannerTrajectory.java
    public PathPlannerTrajectory transformTrajectoryForAlliance(PathPlannerTrajectory trajectory, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return trajectory;
        }

        var transformedStates = new ArrayList<State>();

        for (State s : trajectory.getStates()) {
            var state = (PathPlannerState) s;
            transformedStates.add(transformStateForAlliance(state, alliance));
        }

        return new PathPlannerTrajectory(
                transformedStates,
                trajectory.getMarkers(),
                trajectory.getStartStopEvent(),
                trajectory.getEndStopEvent(),
                trajectory.fromGUI);
    }

    // https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/PathPlannerTrajectory.java
    public PathPlannerState transformStateForAlliance(PathPlannerState state, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return state;
        }

        var transformedState = new PathPlannerState();

        var transformedTranslation =
                new Translation2d(mFieldWidthMeters - state.poseMeters.getX(), state.poseMeters.getY());
        var transformedHeading = GeometryUtil.flipX(state.poseMeters.getRotation());
        var transformedHolonomicRotation = GeometryUtil.flipX(state.holonomicRotation);

        transformedState.timeSeconds = state.timeSeconds;
        transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
        transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
        transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
        transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
        transformedState.holonomicRotation = transformedHolonomicRotation;
        transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
        // transformedState.curveRadius = state.curveRadius;
        transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
        // transformedState.deltaPos = state.deltaPos;

        return transformedState;
    }
}
