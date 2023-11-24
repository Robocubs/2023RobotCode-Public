package com.team1701.frc2023.autonomous;

import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.subsystems.Drive;
import com.team1701.frc2023.subsystems.Hand;
import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake;
import com.team1701.frc2023.subsystems.Intake.IntakeState;
import com.team1701.frc2023.subsystems.PoseEstimator;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;
import com.team1701.lib.trajectory.PathPlannerTransformer;
import com.team1701.lib.util.Callback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AutonomousModes {
    private static Map<PathPlannerPath, List<PathPlannerTrajectory>> pathGroups = new HashMap<>();
    private static final PathPlannerTransformer transformer = new PathPlannerTransformer(Constants.kFieldWidthMeters);
    private static final Map<String, Callback> events = Map.ofEntries(
            Map.entry("targetConePickup", () -> {
                Superstructure.getInstance().setTargetType(TargetType.FLOOR_PICKUP);
                Hand.getInstance().setHandState(HandState.AUTO_CLOSE);
            }),
            Map.entry("targetConeHigh", () -> {
                Superstructure.getInstance().setTargetType(TargetType.HIGH_CONE_FLOOR);
            }),
            Map.entry("targetCubeHigh", () -> {
                Superstructure.getInstance().setTargetType(TargetType.HIGH_CUBE);
            }),
            Map.entry("retractIntake", () -> {
                if (Intake.getInstance().getIntakeState() == IntakeState.INTAKING) {
                    Intake.getInstance().setIntakeState(IntakeState.STOW);
                }
            }),
            Map.entry("deployIntake", () -> {
                Intake.getInstance().setIntakeState(IntakeState.INTAKING);
            }));

    private AutonomousModes() {}

    public static void initializePaths() {
        pathGroups = PathPlannerPaths.getTrajectoryMap();
        System.out.println("Loaded " + pathGroups.size() + " paths");
    }

    public static AutonomousMode none() {
        return AutonomousModeFactory.createNoopAutonomousMode();
    }

    public static AutonomousMode threePiece(boolean isBump) {
        /*
         * NOTE:
         *
         * For preliminary tests, piece-seeking is only enabled on threePiece autons.
         *
         * Should the tests be successful, full integration shall follow, and each
         * autonomous routine should be tested with piece-seeking enabled.
         *
         * After that point, discussions can be started about cleaning the entire
         * codebase up, and preparing for a full release on GitHub.
         */
        PathPlannerPath pathGroupName;
        var secondTarget = TargetType.MID_CUBE_REVERSE;
        var thirdTarget = TargetType.HIGH_CUBE;
        var alliance = Superstructure.getInstance().getAlliance();

        if (isBump) {
            pathGroupName =
                    alliance == Alliance.Red ? PathPlannerPaths.RedBumpScoreThree : PathPlannerPaths.BlueBumpScoreThree;
        } else {
            pathGroupName = alliance == Alliance.Red
                    ? PathPlannerPaths.RedNoBumpScoreThree
                    : PathPlannerPaths.BlueNoBumpScoreThree;
        }

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return justScore(TargetType.LOW);
        }

        var pathGroup = pathGroups.get(pathGroupName);

        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> {
            mode
                    // Initialize
                    .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                    // Set the arm's final handoff state to the scoring type
                    .run(() -> Superstructure.getInstance().setHandoffCompletionTarget(secondTarget))
                    // Score the first piece by dropping it from the hand
                    .setDesiredHandState(HandState.OPEN, true)
                    .waitSeconds(0.3)
                    // Move the hand out of the way without waiting
                    .setArmTarget(TargetType.HANDOFF_READY, false, false)
                    // Follow the first path and get the second cube
                    .followPath(pathGroup.get(0), events, true)
                    // Follow the next path to go score
                    .followPath(pathGroup.get(1), events)
                    .setWaypoint(getFinalPose(pathGroup.get(1)), false)
                    // Ensure the arm is in the scoring position
                    .setTimedArmTarget(secondTarget, false, 1)
                    // Drop the second cube into the mid area
                    .setDesiredHandState(HandState.OPEN, true)
                    // Move the hand to handoff ready
                    .setArmTarget(TargetType.HANDOFF_READY, false, false)
                    // Set the arm's final handoff state to the scoring type
                    .run(() -> Superstructure.getInstance().setHandoffCompletionTarget(thirdTarget))
                    // Follow the second path and get the third cube
                    .followPath(pathGroup.get(2), events, true)
                    .setWaypoint(getFinalPose(pathGroup.get(2)), false)
                    // Follow the next path to go score
                    .followPath(pathGroup.get(3), events)
                    // Ensure the arm is in the scoring position
                    .setArmTarget(thirdTarget, false, true)
                    // Drop the third cube into the high area
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.HOLD);

            if (!isBump) {
                mode.followPath(pathGroup.get(4), events);
            }

            mode.stopDrive();
        });
    }

    public static AutonomousMode noBumpIntake(NoBumpPathChoice behaviorSelected) {
        PathPlannerPath pathGroupName;
        var alliance = Superstructure.getInstance().getAlliance();

        switch (behaviorSelected) {
            case SCORE_TWO_DOCK_INTAKE:
                pathGroupName = alliance == Alliance.Blue
                        ? PathPlannerPaths.BlueNoBumpScoreTwoDockIntake
                        : PathPlannerPaths.RedNoBumpScoreTwoDockIntake;
                break;
            case SCORE_TWO_INTAKE:
                pathGroupName = alliance == Alliance.Blue
                        ? PathPlannerPaths.BlueNoBumpScoreTwoIntake
                        : PathPlannerPaths.RedNoBumpScoreTwoIntake;
                break;
            default:
                return justScore(TargetType.HIGH_CONE_REVERSE);
        }

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return justScore(TargetType.HIGH_CONE_REVERSE);
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> {
            mode
                    // Initialize
                    .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                    // Set the arm's final handoff state to the scoring type
                    .run(() -> Superstructure.getInstance().setHandoffCompletionTarget(TargetType.HIGH_CUBE_REVERSE))
                    // Score the first piece
                    .setTimedArmTarget(TargetType.HIGH_CONE_REVERSE, 3)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.HANDOFF_READY, false)
                    // Follow the first path and get the cube through the intake. Done automatically by handoff
                    .followPath(pathGroup.get(0), events)
                    // Follow the second path
                    .followPath(pathGroup.get(1), events)
                    .setWaypoint(getFinalPose(pathGroup.get(1)), false)
                    // Score the second piece
                    .setArmTarget(TargetType.HIGH_CUBE_REVERSE, false, true)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.HANDOFF_READY, false)
                    .followPath(pathGroup.get(2), events)
                    .setWaypoint(getFinalPose(pathGroup.get(2)), true);

            if (behaviorSelected == NoBumpPathChoice.SCORE_TWO_DOCK_INTAKE) {
                mode.doAutoBalance();
            } else {
                mode.stopDrive();
            }
        });
    }

    public static AutonomousMode noBump(NoBumpPathChoice pathChoice) {
        PathPlannerPath pathGroupName;
        switch (pathChoice) {
            case SCORE_TWO:
                pathGroupName = PathPlannerPaths.NoBumpScoreTwo;
                break;
            case SCORE_TWO_DOCK:
                pathGroupName = PathPlannerPaths.NoBumpScoreTwoDock;
                break;
            default:
                return justScore(TargetType.HIGH_CUBE_REVERSE);
        }

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return justScore(TargetType.HIGH_CUBE_REVERSE);
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> {
            mode
                    // Initialize
                    .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                    // Score first piece
                    .setTimedArmTarget(TargetType.HIGH_CUBE_REVERSE, 3)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.FLOOR_PICKUP, true)
                    // Get second piece
                    .followPath(pathGroup.get(0), events)
                    .setWaypoint(getFinalPose(pathGroup.get(0)), false)
                    .setArmTarget(TargetType.FLOOR_PICKUP, false, true)
                    .setDesiredHandState(HandState.CLOSE, true)
                    // Move to score second piece
                    .setArmTarget(TargetType.HIGH_CONE_FLOOR, true)
                    .followPath(pathGroup.get(1), events)
                    // Score second piece
                    .parallel(
                            branch -> branch.setWaypoint(getFinalPose(pathGroup.get(1)), 1),
                            branch -> branch.setTimedArmTarget(TargetType.HIGH_CONE_FLOOR, 3))
                    .setDesiredHandState(HandState.OPEN, true)
                    // Move to end position
                    .setArmTarget(TargetType.HOLD)
                    .followPath(pathGroup.get(2), events)
                    .setWaypoint(getFinalPose(pathGroup.get(2)), true);

            if (pathChoice == NoBumpPathChoice.SCORE_TWO_DOCK) {
                mode.doAutoBalance();
            } else if (pathChoice == NoBumpPathChoice.SCORE_TWO) {
                mode.stopDrive();
            }
        });
    }

    public static AutonomousMode bump(BumpPathChoice pathChoice) {
        PathPlannerPath pathGroupName;
        switch (pathChoice) {
            case SCORE_TWO:
                pathGroupName = PathPlannerPaths.BumpScoreTwo;
                break;
            case SCORE_TWO_DOCK:
                pathGroupName = PathPlannerPaths.BumpScoreTwoDock;
                break;
            default:
                return justScore(TargetType.HIGH_CUBE_REVERSE);
        }

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return justScore(TargetType.HIGH_CUBE_REVERSE);
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> {
            mode
                    // Initialize
                    .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                    // Score first piece
                    .setTimedArmTarget(TargetType.HIGH_CUBE_REVERSE, 3)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.FLOOR_PICKUP, true)
                    // Get second piece
                    .followPath(pathGroup.get(0), events)
                    .setWaypoint(getFinalPose(pathGroup.get(0)), false)
                    .setArmTarget(TargetType.FLOOR_PICKUP, false, true)
                    .setDesiredHandState(HandState.CLOSE, true)
                    // Move to score second piece
                    .setArmTarget(TargetType.HIGH_CONE_FLOOR, true)
                    .followPath(pathGroup.get(1), events)
                    // Score second piece
                    .parallel(
                            branch -> branch.setWaypoint(getFinalPose(pathGroup.get(1)), 1),
                            branch -> branch.setTimedArmTarget(TargetType.HIGH_CONE_FLOOR, 3))
                    .setDesiredHandState(HandState.OPEN, true)
                    // Move to end position
                    .setArmTarget(TargetType.HOLD)
                    .followPath(pathGroup.get(2), events)
                    .setWaypoint(getFinalPose(pathGroup.get(2)), true);

            if (pathChoice == BumpPathChoice.SCORE_TWO_DOCK) {
                mode.doAutoBalance();
            } else if (pathChoice == BumpPathChoice.SCORE_TWO) {
                mode.stopDrive();
            }
        });
    }

    public static AutonomousMode bumpIntake(BumpPathChoice pathChoice) {
        /*
         * NOTE: This method is almost identical to the noBumpIntake method. The two can probably be merged,
         * but it might be beneficial to have them seperated to make bump-specific tweaks. If not, TODO: combine them.
         */
        PathPlannerPath pathGroupName;
        var alliance = Superstructure.getInstance().getAlliance();

        switch (pathChoice) {
            case SCORE_TWO_INTAKE:
                pathGroupName = alliance == Alliance.Red
                        ? PathPlannerPaths.RedBumpScoreTwoIntake
                        : PathPlannerPaths.BlueBumpScoreTwoIntake;
                break;
            case SCORE_TWO_DOCK_INTAKE:
                pathGroupName = alliance == Alliance.Red
                        ? PathPlannerPaths.RedBumpScoreTwoDockIntake
                        : PathPlannerPaths.BlueBumpScoreTwoDockIntake;
                break;
            default:
                return justScore(TargetType.HIGH_CONE_REVERSE);
        }

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return justScore(TargetType.HIGH_CONE_REVERSE);
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> {
            mode
                    // Initialize
                    .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                    // Set the arm's final handoff state to the scoring type
                    .run(() -> Superstructure.getInstance().setHandoffCompletionTarget(TargetType.HIGH_CUBE_REVERSE))
                    // Score first piece
                    .setTimedArmTarget(TargetType.HIGH_CONE_REVERSE, 3)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.HANDOFF_READY, false)
                    // Follow the first path and get the cube through the intake. Done automatically by handoff
                    .followPath(pathGroup.get(0), events)
                    .setWaypoint(getFinalPose(pathGroup.get(0)), false)
                    // Follow the second path
                    .followPath(pathGroup.get(1), events)
                    .setWaypoint(getFinalPose(pathGroup.get(1)), false)
                    .setDesiredHandState(HandState.CLOSE, true)
                    // Score the second piece
                    .setTimedArmTarget(TargetType.HIGH_CUBE_REVERSE, 1)
                    .setDesiredHandState(HandState.OPEN, true)
                    .setArmTarget(TargetType.HANDOFF_READY, false)
                    .followPath(pathGroup.get(2), events)
                    .setWaypoint(getFinalPose(pathGroup.get(2)), true);

            if (pathChoice == BumpPathChoice.SCORE_TWO_DOCK_INTAKE) {
                mode.doAutoBalance();
            } else {
                mode.stopDrive();
            }
        });
    }

    public static AutonomousMode centerDock() {
        var pathGroupName = PathPlannerPaths.CenterDock;

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return null;
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> mode
                // Initialize
                .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                // Score piece
                .setTimedArmTarget(TargetType.HIGH_CUBE, 4)
                .setDesiredHandState(HandState.OPEN, true)
                .setArmTarget(Superstructure.TargetType.HOLD, false, false)
                // Follow path over charge station
                .followPath(pathGroup.get(0), events)
                .waitSeconds(0.5)
                // Dock
                .followPath(pathGroup.get(1), events)
                .doAutoBalance());
    }

    public static AutonomousMode centerDockPickup() {
        var pathGroupName = PathPlannerPaths.CenterDockPickup;

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return null;
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> mode
                // Initialize
                .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                // Score piece
                .setTimedArmTarget(TargetType.HIGH_CUBE, 4)
                .setDesiredHandState(HandState.OPEN, true)
                .setArmTarget(Superstructure.TargetType.HOLD, false, false)
                // Follow path over charge station
                .followPath(pathGroup.get(0), events)
                .waitSeconds(0.2)
                // Dock
                .followPath(pathGroup.get(1), events)
                .doAutoBalance());
    }

    public static AutonomousMode centerDockScoreTwo() {
        var alliance = Superstructure.getInstance().getAlliance();
        var pathGroupName = alliance == Alliance.Red
                ? PathPlannerPaths.RedCenterDockScoreTwo
                : PathPlannerPaths.BlueCenterDockScoreTwo;

        if (!checkForPathGroups(pathGroupName)) {
            System.out.println("ERROR: PATH NOT LOADED");
            return null;
        }

        var pathGroup = pathGroups.get(pathGroupName);
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> mode
                // Initialize
                .run(() -> setRobotPoseToInitialPose(pathGroup.get(0)))
                // Score piece
                .setTimedArmTarget(TargetType.MID_CUBE, 4)
                .setDesiredHandState(HandState.OPEN, true)
                .setArmTarget(Superstructure.TargetType.HOLD, false, false)
                // .run(() -> {
                //     Superstructure.getInstance().setHandoffCompletionTarget(TargetType.HIGH_CUBE);
                // })
                // Follow paths over charge station
                .followPath(pathGroup.get(0), events)
                .followPath(pathGroup.get(1), events)
                .followPath(pathGroup.get(2), events)
                .followPath(pathGroup.get(3), events)
                .followPath(pathGroup.get(4), events)
                // Score piece
                .setTimedArmTarget(TargetType.HIGH_CUBE, false, 3)
                .setDesiredHandState(HandState.OPEN, true)
                .setArmTarget(TargetType.HOLD, false, false)
                // Dock
                .followPath(pathGroup.get(5), events)
                .followPath(pathGroup.get(6), events)
                .doAutoBalance());
    }

    public static AutonomousMode justScore(TargetType target) {
        return AutonomousModeFactory.createOrderedListAutonomousMode(mode -> mode.setTimedArmTarget(target, 4)
                .setDesiredHandState(HandState.OPEN, true)
                .waitSeconds(1)
                .setTimedArmTarget(Superstructure.TargetType.HOLD, 3));
    }

    private static void setRobotPoseToInitialPose(PathPlannerTrajectory trajectory) {
        updateFieldPosition(getInitialPose(trajectory));
    }

    private static Pose2d getInitialPose(PathPlannerTrajectory trajectory) {
        var initialState = transformer.transformStateForAlliance(
                trajectory.getInitialState(), Superstructure.getInstance().getAlliance());
        return new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    }

    private static Pose2d getFinalPose(PathPlannerTrajectory trajectory) {
        var endState = transformer.transformStateForAlliance(
                trajectory.getEndState(), Superstructure.getInstance().getAlliance());
        return new Pose2d(endState.poseMeters.getTranslation(), endState.holonomicRotation);
    }

    private static boolean checkForPathGroups(PathPlannerPath... trajectoryNames) {
        return Arrays.stream(trajectoryNames).allMatch(pathGroups::containsKey);
    }

    private static void updateFieldPosition(Pose2d pose) {
        Drive.getInstance().zeroGyroscope(pose.getRotation());
        PoseEstimator.getInstance().setPose(pose);
    }

    private static class PathPlannerPaths {
        private static PathConstraints defaultConstraints =
                new PathConstraints(Constants.kMaxAutonomousVelocity, Constants.kMaxAutonomousAcceleration);
        private static PathConstraints fastConstraints = new PathConstraints(3.5, 2.75);
        private static PathConstraints chargeStationConstraints = new PathConstraints(
                Math.min(Constants.kMaxAutonomousVelocity, 1), Constants.kMaxAutonomousAcceleration);
        private static PathConstraints sideChargeStationConstraints = new PathConstraints(
                Math.min(Constants.kMaxAutonomousVelocity, 2), Constants.kMaxAutonomousAcceleration);

        // NO INTAKE PATHS
        private static final PathPlannerPath CenterDockPickup =
                new PathPlannerPath("CenterDockPickupIntake", chargeStationConstraints);
        private static final PathPlannerPath CenterDock = new PathPlannerPath("CenterDock", chargeStationConstraints);
        private static final PathPlannerPath BumpScoreTwoDock = new PathPlannerPath(
                "BumpScoreTwoDock", defaultConstraints, defaultConstraints, chargeStationConstraints);
        private static final PathPlannerPath BumpScoreTwo = new PathPlannerPath("BumpScoreTwo", defaultConstraints);
        private static final PathPlannerPath NoBumpScoreTwoDock = new PathPlannerPath(
                "NoBumpScoreTwoDock", defaultConstraints, defaultConstraints, chargeStationConstraints);
        private static final PathPlannerPath NoBumpScoreTwo = new PathPlannerPath("NoBumpScoreTwo", defaultConstraints);

        /* START BLUE PATHS */

        // INTAKE PATHS
        private static final PathPlannerPath BlueCenterDockScoreTwo = new PathPlannerPath(
                "BlueCenterDockScoreTwo",
                fastConstraints,
                chargeStationConstraints,
                fastConstraints,
                fastConstraints,
                chargeStationConstraints,
                fastConstraints,
                chargeStationConstraints);
        private static final PathPlannerPath BlueNoBumpScoreTwoIntake =
                new PathPlannerPath("BlueNoBumpScoreTwoIntake", defaultConstraints);
        private static final PathPlannerPath BlueNoBumpScoreTwoDockIntake = new PathPlannerPath(
                "BlueNoBumpScoreTwoDockIntake", defaultConstraints, defaultConstraints, chargeStationConstraints);
        private static final PathPlannerPath BlueBumpScoreTwoIntake =
                new PathPlannerPath("BlueBumpScoreTwoIntake", defaultConstraints);
        private static final PathPlannerPath BlueBumpScoreTwoDockIntake = new PathPlannerPath(
                "BlueBumpScoreTwoDockIntake", defaultConstraints, defaultConstraints, sideChargeStationConstraints);
        private static final PathPlannerPath BlueNoBumpScoreThree =
                new PathPlannerPath("BlueNoBumpScoreThree", fastConstraints);
        private static final PathPlannerPath BlueBumpScoreThree =
                new PathPlannerPath("BlueBumpScoreThree", fastConstraints);

        /* END BLUE PATHS */

        /* START RED PATHS */

        // INTAKE PATHS
        private static final PathPlannerPath RedCenterDockScoreTwo = new PathPlannerPath(
                "RedCenterDockScoreTwo",
                fastConstraints,
                chargeStationConstraints,
                fastConstraints,
                fastConstraints,
                chargeStationConstraints,
                fastConstraints,
                chargeStationConstraints);
        private static final PathPlannerPath RedNoBumpScoreTwoIntake =
                new PathPlannerPath("RedNoBumpScoreTwoIntake", defaultConstraints);
        private static final PathPlannerPath RedNoBumpScoreTwoDockIntake = new PathPlannerPath(
                "RedNoBumpScoreTwoDockIntake", defaultConstraints, defaultConstraints, chargeStationConstraints);
        private static final PathPlannerPath RedBumpScoreTwoIntake =
                new PathPlannerPath("RedBumpScoreTwoIntake", defaultConstraints);
        private static final PathPlannerPath RedBumpScoreTwoDockIntake = new PathPlannerPath(
                "RedBumpScoreTwoDockIntake", defaultConstraints, defaultConstraints, sideChargeStationConstraints);
        private static final PathPlannerPath RedNoBumpScoreThree =
                new PathPlannerPath("RedNoBumpScoreThree", fastConstraints);
        private static final PathPlannerPath RedBumpScoreThree =
                new PathPlannerPath("RedBumpScoreThree", fastConstraints);

        /* END RED PATHS */

        private static List<PathPlannerPath> all() {
            return Arrays.stream(PathPlannerPaths.class.getDeclaredFields())
                    .filter(field -> Modifier.isStatic(field.getModifiers())
                            && PathPlannerPath.class.isAssignableFrom(field.getType()))
                    .map(field -> {
                        try {
                            return (PathPlannerPath) field.get(null);
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            return null;
                        }
                    })
                    .filter(field -> field != null)
                    .distinct()
                    .collect(Collectors.toUnmodifiableList());
        }

        private static Map<PathPlannerPath, List<PathPlannerTrajectory>> getTrajectoryMap() {
            return PathPlannerPaths.all().stream().collect(Collectors.toUnmodifiableMap(path -> path, path -> {
                System.out.println("Loading " + path.name + ".path");
                var firstConstraint = path.constraints.length > 0 ? path.constraints[0] : defaultConstraints;
                var remainingConstraints = path.constraints.length > 1
                        ? Arrays.copyOfRange(path.constraints, 1, path.constraints.length)
                        : new PathConstraints[] {};
                return PathPlanner.loadPathGroup(path.name, false, firstConstraint, remainingConstraints);
            }));
        }
    }

    private static class PathPlannerPath {
        private final String name;
        private final PathConstraints[] constraints;
        private final int hashCode;

        private PathPlannerPath(String name, PathConstraints... constraints) {
            this.name = name;
            this.constraints = constraints;
            var toHash = Stream.concat(
                            Stream.of(name),
                            Stream.of(constraints)
                                    .flatMap(constraint ->
                                            Stream.of(constraint.maxVelocity, constraint.maxAcceleration)))
                    .toArray(Object[]::new);
            this.hashCode = Objects.hash(toHash);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) {
                return true;
            }

            if (o == null || getClass() != o.getClass()) {
                return false;
            }

            var that = (PathPlannerPath) o;
            return name == that.name && constraintsEqual(that.constraints);
        }

        private boolean constraintsEqual(PathConstraints[] constraints) {
            if (constraints.length != this.constraints.length) {
                return false;
            }

            for (int i = 0; i < constraints.length; i++) {
                if (constraints[i].maxVelocity != this.constraints[i].maxVelocity
                        && constraints[i].maxAcceleration != this.constraints[i].maxAcceleration) {
                    return false;
                }
            }

            return true;
        }

        @Override
        public int hashCode() {
            return this.hashCode;
        }
    }

    public enum NoBumpPathChoice {
        SCORE_TWO,
        SCORE_TWO_INTAKE,
        SCORE_TWO_DOCK,
        SCORE_TWO_DOCK_INTAKE
    }

    public enum BumpPathChoice {
        SCORE_TWO,
        SCORE_TWO_INTAKE,
        SCORE_TWO_DOCK,
        SCORE_TWO_DOCK_INTAKE
    }
}
