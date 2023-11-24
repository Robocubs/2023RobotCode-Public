package com.team1701.frc2023.autonomous.states;

import java.util.function.Consumer;

public class StateGroupState extends State {

    private final StateGroup mGroup;

    private StateGroupState(Configuration configuration) {
        super(configuration.name, configuration.next);

        mGroup = configuration.group;
    }

    public StateGroupState(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    @Override
    public void start() {
        mGroup.start();
    }

    @Override
    public void update() {
        mGroup.update();
    }

    @Override
    public void stop() {
        mGroup.stop();
    }

    @Override
    public boolean isCompleted() {
        return mGroup.isCompleted();
    }

    public static class Configuration {
        private String name;
        private String next;
        private StateGroup group;

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

        public Configuration group(Consumer<StateGroup.Configuration> group) {
            this.group = new StateGroup(group);
            return this;
        }
    }
}
