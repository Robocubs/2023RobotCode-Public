package com.team1701.frc2023.autonomous.states;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ChoiceState extends State {

    private final Supplier<String> mChooser;

    private ChoiceState(Configuration configuration) {
        super(configuration.name, null);

        mChooser = configuration.chooser;
    }

    public ChoiceState(Consumer<Configuration> config) {
        this(new Configuration(config));
    }

    @Override
    public String getNext() {
        return mChooser.get();
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
        private Supplier<String> chooser;

        private Configuration() {}

        private Configuration(Consumer<Configuration> config) {
            config.accept(this);
        }

        public Configuration name(String name) {
            this.name = name;
            return this;
        }

        public Configuration choice(Supplier<String> chooser) {
            this.chooser = chooser;
            return this;
        }
    }
}
