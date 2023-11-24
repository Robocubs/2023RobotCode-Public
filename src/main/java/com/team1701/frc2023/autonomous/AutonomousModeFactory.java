package com.team1701.frc2023.autonomous;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.autonomous.states.ChoiceState;
import com.team1701.frc2023.autonomous.states.EndState;
import com.team1701.frc2023.autonomous.states.ParallelState;
import com.team1701.frc2023.autonomous.states.State;
import com.team1701.frc2023.autonomous.states.StateGroup;
import com.team1701.frc2023.autonomous.states.StateGroupState;
import com.team1701.frc2023.autonomous.states.TaskState;
import com.team1701.frc2023.autonomous.tasks.ArmTargetTask;
import com.team1701.frc2023.autonomous.tasks.AutoBalanceTask;
import com.team1701.frc2023.autonomous.tasks.HandTask;
import com.team1701.frc2023.autonomous.tasks.IntakeTargetTask;
import com.team1701.frc2023.autonomous.tasks.PathFollowingTask;
import com.team1701.frc2023.autonomous.tasks.SimpleTask;
import com.team1701.frc2023.autonomous.tasks.Task;
import com.team1701.frc2023.autonomous.tasks.TimedArmTargetTask;
import com.team1701.frc2023.autonomous.tasks.TimedDriveTask;
import com.team1701.frc2023.autonomous.tasks.TimedWaypointTask;
import com.team1701.frc2023.autonomous.tasks.WaitTask;
import com.team1701.frc2023.autonomous.tasks.WaypointTask;
import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake.IntakeState;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;
import com.team1701.lib.trajectory.PathPlannerTransformer;
import com.team1701.lib.util.Callback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public final class AutonomousModeFactory {

    private AutonomousModeFactory() {}

    public static AutonomousMode createNoopAutonomousMode() {
        return new AutonomousMode() {

            @Override
            public void start() {}

            @Override
            public void loop() {}

            @Override
            public void stop() {}

            @Override
            public boolean isRunning() {
                return false;
            }
        };
    }

    public static AutonomousMode createOrderedListAutonomousMode(
            Consumer<OrderedListAutonomousModeConfiguration> config) {
        var configuration = new OrderedListAutonomousModeConfiguration(config);
        return new StateMachineAutonomousMode(new StateGroup(configuration.toStateGroupConfig()));
    }

    public static AutonomousMode createStateMachineAutonomousMode(Consumer<StateGroup.Configuration> config) {
        return new StateMachineAutonomousMode(config);
    }

    public static class OrderedListAutonomousModeConfiguration {

        private List<State> states = new ArrayList<>();
        private PathPlannerTransformer pathPlannerTransformer = new PathPlannerTransformer(Constants.kFieldWidthMeters);

        private OrderedListAutonomousModeConfiguration() {}

        private OrderedListAutonomousModeConfiguration(Consumer<OrderedListAutonomousModeConfiguration> config) {
            config.accept(this);
            run(() -> System.out.println("Autonomous completed with " + Timer.getMatchTime() + " seconds remaining."));
        }

        private Consumer<StateGroup.Configuration> toStateGroupConfig() {
            return config -> config.startAt("0")
                    .states(Stream.concat(states.stream(), Stream.of(new EndState(end -> end.name(getNewStateName()))))
                            .toArray(State[]::new));
        }

        private String getNewStateName() {
            return Integer.toString(states.size());
        }

        private String getNewStateNext() {
            return Integer.toString(states.size() + 1);
        }

        private String getFutureState(int stepsAfterNew) {
            return Integer.toString(states.size() + stepsAfterNew);
        }

        public OrderedListAutonomousModeConfiguration run(Consumer<OrderedListAutonomousModeConfiguration> config) {
            states.add(new StateGroupState(group -> group.name(getNewStateName())
                    .next(getNewStateNext())
                    .group(new OrderedListAutonomousModeConfiguration(config).toStateGroupConfig())));

            return this;
        }

        public OrderedListAutonomousModeConfiguration setWaypoint(Pose2d pose) {
            return setWaypoint(pose, true);
        }

        public OrderedListAutonomousModeConfiguration setWaypoint(Pose2d pose, boolean waitForWaypoint) {
            return task(new WaypointTask(pose, waitForWaypoint));
        }

        public OrderedListAutonomousModeConfiguration setWaypoint(Pose2d pose, double maxDuration) {
            return task(new TimedWaypointTask(pose, maxDuration));
        }

        @SafeVarargs
        public final OrderedListAutonomousModeConfiguration parallel(
                Consumer<OrderedListAutonomousModeConfiguration>... branches) {
            return parallel(0, branches);
        }

        @SafeVarargs
        public final OrderedListAutonomousModeConfiguration parallel(
                int minimumBranchesToComplete, Consumer<OrderedListAutonomousModeConfiguration>... branches) {

            @SuppressWarnings("unchecked")
            Consumer<StateGroup.Configuration>[] pBranches = Arrays.stream(branches)
                    .map(OrderedListAutonomousModeConfiguration::new)
                    .map(OrderedListAutonomousModeConfiguration::toStateGroupConfig)
                    .toArray(Consumer[]::new);

            states.add(new ParallelState(parallel -> parallel.name(getNewStateName())
                    .next(getNewStateNext())
                    .minimumBranchesToComplete(minimumBranchesToComplete)
                    .branches(pBranches)));

            return this;
        }

        public OrderedListAutonomousModeConfiguration loop(
                Supplier<Boolean> predicate, Consumer<OrderedListAutonomousModeConfiguration> loop) {

            var choiceStateName = getNewStateName();
            var groupStateName = getFutureState(1);
            var loopExitStateName = getFutureState(2);

            states.add(new ChoiceState(choice ->
                    choice.name(choiceStateName).choice(() -> predicate.get() ? groupStateName : loopExitStateName)));

            states.add(new StateGroupState(group -> group.name(groupStateName)
                    .next(choiceStateName)
                    .group(new OrderedListAutonomousModeConfiguration(loop).toStateGroupConfig())));

            return this;
        }

        public OrderedListAutonomousModeConfiguration choice(
                Supplier<Boolean> predicate, Consumer<OrderedListAutonomousModeConfiguration> then) {
            var choiceStateName = getNewStateName();
            var groupStateName = getFutureState(1);
            var exitStateName = getFutureState(2);

            states.add(new ChoiceState(choice ->
                    choice.name(choiceStateName).choice(() -> predicate.get() ? groupStateName : exitStateName)));

            states.add(new StateGroupState(group -> group.name(groupStateName)
                    .next(exitStateName)
                    .group(new OrderedListAutonomousModeConfiguration(then).toStateGroupConfig())));

            return this;
        }

        public OrderedListAutonomousModeConfiguration task(Task task) {
            states.add(new TaskState(config ->
                    config.name(getNewStateName()).next(getNewStateNext()).task(task)));

            return this;
        }

        public OrderedListAutonomousModeConfiguration run(Callback task) {
            return task(new SimpleTask() {
                @Override
                public void start() {
                    task.invoke();
                }
            });
        }

        public OrderedListAutonomousModeConfiguration stopDrive() {
            return task(new TimedDriveTask(new ChassisSpeeds(), 0));
        }

        public OrderedListAutonomousModeConfiguration timedDrive(ChassisSpeeds s, double duration) {
            return task(new TimedDriveTask(s, duration));
        }

        public OrderedListAutonomousModeConfiguration waitSeconds(double duration) {
            return task(new WaitTask(duration));
        }

        public OrderedListAutonomousModeConfiguration followPath(PathPlannerTrajectory trajectory) {
            return followPath(trajectory, true);
        }

        public OrderedListAutonomousModeConfiguration followPath(
                PathPlannerTrajectory trajectory, boolean transformForAlliance) {
            final var transformedTrajectory = transformForAlliance
                    ? pathPlannerTransformer.transformTrajectoryForAlliance(
                            trajectory, Superstructure.getInstance().getAlliance())
                    : trajectory;
            return task(new PathFollowingTask(() -> transformedTrajectory));
        }

        public OrderedListAutonomousModeConfiguration followPath(
                PathPlannerTrajectory trajectory, Map<String, Callback> eventCallbacks) {
            return followPath(trajectory, true, eventCallbacks);
        }

        public OrderedListAutonomousModeConfiguration followPath(
                PathPlannerTrajectory trajectory, Map<String, Callback> eventCallbacks, boolean pieceSeekingEnabled) {
            return followPath(trajectory, true, eventCallbacks, pieceSeekingEnabled);
        }

        public OrderedListAutonomousModeConfiguration followPath(
                PathPlannerTrajectory trajectory, boolean transformForAlliance, Map<String, Callback> eventCallbacks) {
            final var transformedTrajectory = transformForAlliance
                    ? pathPlannerTransformer.transformTrajectoryForAlliance(
                            trajectory, Superstructure.getInstance().getAlliance())
                    : trajectory;
            return task(new PathFollowingTask(() -> transformedTrajectory, eventCallbacks));
        }

        public OrderedListAutonomousModeConfiguration followPath(
                PathPlannerTrajectory trajectory,
                boolean transformForAlliance,
                Map<String, Callback> eventCallbacks,
                boolean pieceSeekingEnabled) {
            final var transformedTrajectory = transformForAlliance
                    ? pathPlannerTransformer.transformTrajectoryForAlliance(
                            trajectory, Superstructure.getInstance().getAlliance())
                    : trajectory;
            return task(new PathFollowingTask(() -> transformedTrajectory, eventCallbacks, pieceSeekingEnabled));
        }

        public OrderedListAutonomousModeConfiguration setTimedArmTarget(TargetType type, double maxDuration) {
            return setTimedArmTarget(type, false, 0, maxDuration);
        }

        public OrderedListAutonomousModeConfiguration setTimedArmTarget(
                TargetType type, boolean premove, double maxDuration) {
            return setTimedArmTarget(type, premove, 0, maxDuration);
        }

        public OrderedListAutonomousModeConfiguration setTimedArmTarget(
                TargetType type, boolean premove, double minDuration, double maxDuration) {
            return task(new TimedArmTargetTask(type, premove, minDuration, maxDuration));
        }

        public OrderedListAutonomousModeConfiguration setArmTarget(TargetType type) {
            return setArmTarget(type, false, false);
        }

        public OrderedListAutonomousModeConfiguration setArmTarget(TargetType type, boolean premove) {
            return setArmTarget(type, premove, false);
        }

        public OrderedListAutonomousModeConfiguration setArmTarget(
                TargetType type, boolean premove, boolean waitForPosition) {
            return task(new ArmTargetTask(type, premove, waitForPosition));
        }

        public OrderedListAutonomousModeConfiguration setDesiredHandState(HandState state, boolean waitForActuation) {
            return task(new HandTask(state, waitForActuation));
        }

        public OrderedListAutonomousModeConfiguration doAutoBalance() {
            return task(new AutoBalanceTask());
        }

        public OrderedListAutonomousModeConfiguration setIntakeState(IntakeState state, boolean wait) {
            return task(new IntakeTargetTask(state, wait));
        }
    }

    private static class StateMachineAutonomousMode implements AutonomousMode {

        private StateGroup mStates;

        private StateMachineAutonomousMode(StateGroup states) {
            mStates = states;
        }

        private StateMachineAutonomousMode(Consumer<StateGroup.Configuration> config) {
            this(new StateGroup(config));
        }

        @Override
        public void start() {
            System.out.println("Started Autonomous Mode");
            mStates.start();
        }

        @Override
        public void loop() {
            mStates.update();
        }

        @Override
        public void stop() {
            mStates.stop();
            System.out.println("Stopped Autonomous Mode");
        }

        @Override
        public boolean isRunning() {
            return !mStates.isCompleted();
        }
    }
}
