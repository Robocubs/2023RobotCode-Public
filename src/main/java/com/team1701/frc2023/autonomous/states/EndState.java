package com.team1701.frc2023.autonomous.states;

import java.util.function.Consumer;

public class EndState extends State {

    private EndState(Configuration configuration) {
        super(configuration.name, null);
    }

    public EndState(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public boolean isCompleted() {
        return true;
    }

    public static class Configuration {
        private String name;

        private Configuration() {}

        private Configuration(Consumer<Configuration> config) {
            config.accept(this);
        }

        public Configuration name(String name) {
            this.name = name;
            return this;
        }
    }
}
