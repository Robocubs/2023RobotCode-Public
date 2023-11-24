package com.team1701.frc2023.autonomous.states;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class ParallelState extends State {

    private final List<StateGroup> mBranches;
    private final int mMinimumBranchesToComplete;
    private boolean mRunning;

    private ParallelState(Configuration configuration) {
        super(configuration.name, configuration.next);

        mBranches = configuration.branches;
        mMinimumBranchesToComplete = configuration.minimumBranchesToComplete;
    }

    public ParallelState(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    @Override
    public void start() {
        if (mRunning) {
            return;
        }

        mBranches.forEach(StateGroup::start);
        mRunning = true;
    }

    @Override
    public void update() {
        mBranches.forEach(StateGroup::update);

        if ((mMinimumBranchesToComplete == 0 && mBranches.stream().allMatch(StateGroup::isCompleted))
                || (mBranches.stream().filter(StateGroup::isCompleted).count() >= mMinimumBranchesToComplete)) {
            mRunning = false;
        }
    }

    @Override
    public void stop() {
        mBranches.forEach(StateGroup::stop);
    }

    @Override
    public boolean isCompleted() {
        return !mRunning;
    }

    public static class Configuration {
        private String name;
        private String next;
        private List<StateGroup> branches;
        private int minimumBranchesToComplete = 0;

        private Configuration() {}

        private Configuration(Consumer<Configuration> config) {
            config.accept(this);
        }

        @SafeVarargs
        public final Configuration branches(Consumer<StateGroup.Configuration>... branches) {
            this.branches = Arrays.stream(branches).map(StateGroup::new).collect(Collectors.toUnmodifiableList());
            return this;
        }

        public final Configuration branches(List<Consumer<StateGroup.Configuration>> branches) {
            this.branches = branches.stream().map(StateGroup::new).collect(Collectors.toUnmodifiableList());
            return this;
        }

        public Configuration name(String name) {
            this.name = name;
            return this;
        }

        public Configuration next(String next) {
            this.next = next;
            return this;
        }

        public Configuration minimumBranchesToComplete(int minimumBranchesToComplete) {
            this.minimumBranchesToComplete = minimumBranchesToComplete;
            return this;
        }
    }
}
