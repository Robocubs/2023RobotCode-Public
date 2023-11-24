package com.team1701.frc2023.autonomous.states;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.team1701.frc2023.autonomous.tasks.PathFollowingTask;
import com.team1701.frc2023.autonomous.tasks.SimpleTask;
import com.team1701.frc2023.autonomous.tasks.Task;
import com.team1701.frc2023.autonomous.tasks.TimedDriveTask;
import com.team1701.frc2023.autonomous.tasks.WaitTask;
import com.team1701.lib.util.Callback;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class TaskState extends State {

    private final Task mTask;

    private TaskState(Configuration configuration) {
        super(configuration.name, configuration.next);
        mTask = configuration.task;
    }

    public TaskState(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    @Override
    public void start() {
        System.out.println("Started task " + getName() + " [" + mTask.getClass().getSimpleName() + "] "
                + Timer.getFPGATimestamp());
        mTask.start();
    }

    @Override
    public void update() {
        mTask.update();
    }

    @Override
    public void stop() {
        mTask.stop();
        System.out.println(
                "Stopped task " + getName() + " [" + mTask.getClass().getSimpleName() + "]" + Timer.getFPGATimestamp());
    }

    @Override
    public boolean isCompleted() {
        return mTask.isCompleted();
    }

    public static class Configuration {
        private String name;
        private String next;
        private Task task;

        private Configuration() {}

        private Configuration(Consumer<Configuration> config) {
            config.accept(this);
        }

        public Configuration name(String name) {
            this.name = name;
            return this;
        }

        public Configuration next(String next) {
            this.next = next;
            return this;
        }

        public Configuration task(Task task) {
            this.task = task;
            return this;
        }

        public Configuration run(Callback task) {
            this.task = new SimpleTask() {
                @Override
                public void start() {
                    task.invoke();
                }
            };
            return this;
        }

        public Configuration timedDrive(ChassisSpeeds s, double duration) {
            this.task = new TimedDriveTask(s, duration);
            return this;
        }

        public Configuration waitSeconds(double duration) {
            this.task = new WaitTask(duration);
            return this;
        }

        public Configuration followPath(PathPlannerTrajectory trajectory) {
            this.task = new PathFollowingTask(() -> trajectory);
            return this;
        }

        public Configuration followPath(Supplier<PathPlannerTrajectory> trajectorySupplier) {
            this.task = new PathFollowingTask(trajectorySupplier);
            return this;
        }
    }
}
