package com.team1701.lib.arm;

import java.util.LinkedList;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.team1701.lib.arm.ArmSetpointGenerator.PassDirection;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ArmSetpointGeneratorTest {

    private static final Rotation2d angleToFront = Rotation2d.fromDegrees(30);
    private static final Rotation2d angleToBack = Rotation2d.fromDegrees(-25);
    private static double maxExtension = 10;
    private static double maxExtensionPassing = 5;
    private static Rotation2d measuredRotationErrorMargin = Rotation2d.fromDegrees(1);
    private static double measuredExtensionErrorMargin = 0.01;
    private static ArmKinematics kinematics =
            new ArmKinematics(new Pose2d(0.2, 1d, GeometryUtil.kRotationIdentity), new Translation2d(0.03, 1d));

    private ArmSetpointGenerator createDefaultGenerator() {
        return new ArmSetpointGenerator(
                angleToFront,
                angleToBack,
                kinematics,
                PassDirection.THROUGH,
                maxExtension,
                maxExtensionPassing,
                measuredRotationErrorMargin,
                measuredExtensionErrorMargin);
    }

    private void assertPathDoesNotCollideWithRobot(ArmSetpoint startSetpoint, ArmSetpoint endSetpoint) {
        if (startSetpoint.extension < measuredExtensionErrorMargin
                && endSetpoint.extension < measuredExtensionErrorMargin) {
            return;
        }

        if (MathUtil.angleModulus(startSetpoint.angle.getRadians()) >= MathUtil.angleModulus(angleToFront.getRadians())
                && MathUtil.angleModulus(endSetpoint.angle.getRadians())
                        >= MathUtil.angleModulus(angleToFront.getRadians())) {
            return;
        }

        if (MathUtil.angleModulus(startSetpoint.angle.getRadians()) <= MathUtil.angleModulus(angleToBack.getRadians())
                && MathUtil.angleModulus(endSetpoint.angle.getRadians())
                        <= MathUtil.angleModulus(angleToBack.getRadians())) {
            return;
        }

        assertFalse(true, "Path from " + startSetpoint + " to " + endSetpoint + " collides with robot");
    }

    private void moveToGoal(ArmSetpoint setpoint, ArmSetpoint goal, ArmSetpointGenerator generator) {
        System.out.println("Moving arm from " + setpoint + " to " + goal);
        var i = 0;
        while (i++ < 100 && !setpoint.equals(goal)) {
            var newSetpoint = generator.generateSetpoint(setpoint, goal);
            System.out.println(newSetpoint);
            assertPathDoesNotCollideWithRobot(setpoint, newSetpoint);
            setpoint = newSetpoint;
        }

        assertEquals(setpoint, goal);
    }

    @Test
    public void testGenerateSetpoint() {
        var generator = createDefaultGenerator();
        var setpoints = Stream.of(
                        new ArmSetpoint(),
                        new ArmSetpoint(Rotation2d.fromDegrees(-110), 1),
                        new ArmSetpoint(Rotation2d.fromDegrees(120), 1),
                        new ArmSetpoint(Rotation2d.fromDegrees(10), 0),
                        new ArmSetpoint(Rotation2d.fromDegrees(-20), 0),
                        new ArmSetpoint(Rotation2d.fromDegrees(50), 0.5),
                        new ArmSetpoint(Rotation2d.fromDegrees(-40), 0.3),
                        new ArmSetpoint())
                .collect(Collectors.toCollection(LinkedList::new));

        var setpoint = setpoints.pop();
        while (!setpoints.isEmpty()) {
            var goal = setpoints.pop();
            moveToGoal(setpoint, goal, generator);
            setpoint = goal;
        }
    }

    @Test
    public void testGenerateSetpointRotatesBeforeExtendingOutward() {
        var generator = createDefaultGenerator();

        var measuredSetpoint = new ArmSetpoint(angleToFront, 0);
        var desiredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        var expectedSetpoint = new ArmSetpoint(desiredSetpoint.angle, measuredSetpoint.extension);
        var actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(30)), 0.2);
        desiredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        expectedSetpoint = new ArmSetpoint(desiredSetpoint.angle, measuredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(30)), 0.2);
        desiredSetpoint = new ArmSetpoint(measuredSetpoint.angle, 0.8);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(30)), 0.2);
        desiredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(90)), measuredSetpoint.extension);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack, 0);
        desiredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        expectedSetpoint = new ArmSetpoint(desiredSetpoint.angle, measuredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(10)), 0.1);
        desiredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        expectedSetpoint = new ArmSetpoint(desiredSetpoint.angle, measuredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(10)), 0.1);
        desiredSetpoint = new ArmSetpoint(measuredSetpoint.angle, 1);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(10)), 0.1);
        desiredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), measuredSetpoint.extension);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testGenerateSetpointRetractsBeforeRotatingInward() {
        var generator = createDefaultGenerator();

        var measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        var desiredSetpoint = new ArmSetpoint(angleToFront, 0);
        var expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, desiredSetpoint.extension);
        var actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        desiredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(30)), 0.2);
        expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, desiredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        desiredSetpoint = new ArmSetpoint(measuredSetpoint.angle, 0.2);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(50)), 0.8);
        desiredSetpoint = new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(30)), measuredSetpoint.extension);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        desiredSetpoint = new ArmSetpoint(angleToBack, 0);
        expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, desiredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        desiredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(10)), 0.1);
        expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, desiredSetpoint.extension);
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        desiredSetpoint = new ArmSetpoint(measuredSetpoint.angle, 1);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);

        measuredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(90)), 1);
        desiredSetpoint = new ArmSetpoint(angleToBack.minus(Rotation2d.fromDegrees(10)), measuredSetpoint.extension);
        expectedSetpoint = desiredSetpoint;
        actualSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        assertEquals(expectedSetpoint, actualSetpoint);
    }

    @Test
    public void testGenerateSetpointWillNotRotateIfExtendedInRobot() {
        var generator = createDefaultGenerator();
        var measuredSetpoint =
                new ArmSetpoint(angleToFront.interpolate(angleToBack, 0.5), measuredExtensionErrorMargin + 0.01);

        // Desired setpoint outside robot
        var desiredSetpoint =
                new ArmSetpoint(angleToFront.plus(Rotation2d.fromDegrees(10)), measuredExtensionErrorMargin + 1);
        var generatedSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        var expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, 0);
        assertEquals(expectedSetpoint, generatedSetpoint);

        // Desired setpoint inside robot
        desiredSetpoint = new ArmSetpoint(angleToFront.interpolate(measuredSetpoint.angle, 0.5), 0);
        generatedSetpoint = generator.generateSetpoint(measuredSetpoint, desiredSetpoint);
        expectedSetpoint = new ArmSetpoint(measuredSetpoint.angle, 0);
        assertEquals(expectedSetpoint, generatedSetpoint);
    }
}
