package com.team1701.frc2023.autonomous.states;

public abstract class State {

    private final String mName;
    private final String mNext;

    public State(String name, String next) {
        mName = name;
        mNext = next;
    }

    public String getName() {
        return mName;
    }

    public String getNext() {
        return mNext;
    }

    public abstract void start();

    public abstract void update();

    public abstract void stop();

    public abstract boolean isCompleted();
}
