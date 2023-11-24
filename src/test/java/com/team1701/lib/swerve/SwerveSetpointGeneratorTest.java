package com.team1701.lib.swerve;

import com.team1701.lib.util.KinematicsUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class SwerveSetpointGeneratorTest {

    private static final double kRobotSide = 0.616; // m
    private static final ExtendedSwerveDriveKinematics kKinematics = new ExtendedSwerveDriveKinematics(
            // Front left
            new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
            // Front right
            new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
            // Back left
            new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
            // Back right
            new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0));
    private static final SwerveSetpointGenerator.KinematicLimits kKinematicLimits =
            new SwerveSetpointGenerator.KinematicLimits();

    static {
        kKinematicLimits.kMaxDriveVelocity = 5.0; // m/s
        kKinematicLimits.kMaxDriveAcceleration = 10.0; // m/s^2
        kKinematicLimits.kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s
    }

    private static final double kDt = 0.01; // s
    private static final double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private static final double kMaxAccelerationError = 0.1; // m/s^2

    private void satisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
        for (int i = 0; i < prev.mModuleStates.length; ++i) {
            final var prevModule = prev.mModuleStates[i];
            final var nextModule = next.mModuleStates[i];
            var diffRotation = prevModule.angle.unaryMinus().rotateBy(nextModule.angle);
            assertTrue(Math.abs(diffRotation.getRadians())
                    < kKinematicLimits.kMaxSteeringVelocity + kMaxSteeringVelocityError);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.kMaxDriveVelocity);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt
                    <= kKinematicLimits.kMaxDriveAcceleration + kMaxAccelerationError);
        }
    }

    private SwerveSetpoint driveToGoal(
            SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
        System.out.println("Driving to goal state " + goal);
        System.out.println("Initial state: " + prevSetpoint);
        while (!KinematicsUtil.toTwist2d(prevSetpoint.mChassisSpeeds).equals(KinematicsUtil.toTwist2d(goal))) {
            var newSetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
            System.out.println(newSetpoint);
            satisfiesConstraints(prevSetpoint, newSetpoint);
            prevSetpoint = newSetpoint;
        }
        return prevSetpoint;
    }

    @Test
    public void testGenerateSetpoint() {
        var initialStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };
        var setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new SwerveSetpointGenerator(kKinematics);

        var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);
    }
}
