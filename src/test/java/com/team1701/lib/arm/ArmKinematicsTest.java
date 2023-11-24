package com.team1701.lib.arm;

import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ArmKinematicsTest {

    @Test
    public void testToArmSetpoint90Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var target = new Translation2d(1.3, 0.9);
        var expectedSetpoint = new ArmSetpoint(Rotation2d.fromDegrees(90), 0.5);
        var actualSetpoint = kinematics.toArmSetpoint(target);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testToArmSetpointMinus90Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var target = new Translation2d(-1.3, 1.1);
        var expectedSetpoint = new ArmSetpoint(Rotation2d.fromDegrees(-90), 1.5);
        var actualSetpoint = kinematics.toArmSetpoint(target);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testToArmSetpointZeroDegrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var target = new Translation2d(0.4, 0.5);
        var expectedSetpoint = new ArmSetpoint(Rotation2d.fromDegrees(0), 0.2);
        var actualSetpoint = kinematics.toArmSetpoint(target);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testToArmSetpoint180Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var target = new Translation2d(0.6, 2);
        var expectedSetpoint = new ArmSetpoint(Rotation2d.fromDegrees(180), 0.7);
        var actualSetpoint = kinematics.toArmSetpoint(target);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testToPose90Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var setpoint = new ArmSetpoint(Rotation2d.fromDegrees(90), 0.5);
        var expectedPosition = new Pose2d(1.3, 0.9, Rotation2d.fromDegrees(90));
        var actualPosition = kinematics.toPose(setpoint);
        assertEquals(expectedPosition, actualPosition);
    }

    @Test
    public void testToPoseMinus90Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var setpoint = new ArmSetpoint(Rotation2d.fromDegrees(-90), 1.5);
        var expectedPosition = new Pose2d(-1.3, 1.1, Rotation2d.fromDegrees(-90));
        var actualPosition = kinematics.toPose(setpoint);
        assertEquals(expectedPosition, actualPosition);
    }

    @Test
    public void testToPoseZeroDegrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var setpoint = new ArmSetpoint(Rotation2d.fromDegrees(0), 0.2);
        var expectedPosition = new Pose2d(0.4, 0.5, Rotation2d.fromDegrees(0));
        var actualPosition = kinematics.toPose(setpoint);
        assertEquals(expectedPosition, actualPosition);
    }

    @Test
    public void testToPose180Degrees() {
        var position = new Pose2d(0.5, 1, GeometryUtil.kRotationHalfPi.unaryMinus());
        var handPosition = new Translation2d(0.3, -0.1);
        var kinematics = new ArmKinematics(position, handPosition);
        var setpoint = new ArmSetpoint(Rotation2d.fromDegrees(180), 0.7);
        var expectedPosition = new Pose2d(0.6, 2, Rotation2d.fromDegrees(180));
        var actualPosition = kinematics.toPose(setpoint);
        assertEquals(expectedPosition, actualPosition);
    }
}
