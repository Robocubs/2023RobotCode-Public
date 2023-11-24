package com.team1701.frc2023.autonomous.states;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class StateGroup {

    private final Map<String, State> mStates;
    private final String mStartAt;
    private State mState = null;
    private boolean mRunning = false;

    private StateGroup(Configuration configuration) {
        mStartAt = configuration.startAt;
        mStates = configuration.states;
    }

    public StateGroup(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    public void start() {
        if (mRunning) {
            return;
        }

        mState = mStates.get(mStartAt);

        Logger.getInstance().recordOutput("Autonomous/State", mState.getName());
        mState.start();
        mRunning = true;
    }

    public void update() {
        if (!mRunning) {
            return;
        }

        if (!mState.isCompleted()) {
            mState.update();
            return;
        }

        // Some states complete synchronously
        while (mState.isCompleted()) {
            mState.stop();

            if (mState.getNext() == null) {
                mRunning = false;
                mState = null;
                return;
            }

            mState = mStates.get(mState.getNext());
            Logger.getInstance().recordOutput("Autonomous/State", mState.getName());
            mState.start();
        }
    }

    public void stop() {
        if (!mRunning) {
            return;
        }

        mState.stop();
        mRunning = false;
    }

    public boolean isCompleted() {
        return !mRunning;
    }

    public static class Configuration {
        private final Map<String, State> states = new HashMap<>();
        private String startAt;

        private Configuration() {}

        private Configuration(Consumer<Configuration> config) {
            config.accept(this);
        }

        public Configuration startAt(String startAt) {
            this.startAt = startAt;
            return this;
        }

        public Configuration states(State... states) {
            Arrays.stream(states).forEach(state -> this.states.put(state.getName(), state));
            return this;
        }

        public Configuration states(Consumer<StateListConfiguration> config) {
            new StateListConfiguration(config).states.forEach(state -> this.states.put(state.getName(), state));
            return this;
        }
    }

    public static class StateListConfiguration {
        private final List<State> states = new ArrayList<>();

        private StateListConfiguration() {}

        private StateListConfiguration(Consumer<StateListConfiguration> config) {
            config.accept(this);
        }

        public StateListConfiguration task(Consumer<TaskState.Configuration> task) {
            states.add(new TaskState(task));
            return this;
        }

        public StateListConfiguration parallel(Consumer<ParallelState.Configuration> parallel) {
            states.add(new ParallelState(parallel));
            return this;
        }

        public StateListConfiguration choice(Consumer<ChoiceState.Configuration> choice) {
            states.add(new ChoiceState(choice));
            return this;
        }
    }
}
