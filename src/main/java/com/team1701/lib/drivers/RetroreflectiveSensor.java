package com.team1701.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class RetroreflectiveSensor {
    private final DigitalInput sensor;
    private boolean reversed;
    private boolean onPassReset;
    private int count;

    public RetroreflectiveSensor(int channel) {
        sensor = new DigitalInput(channel);
    }

    public RetroreflectiveSensor(DigitalInput digitalInput) {
        sensor = digitalInput;
    }

    public DigitalInput getSensor() {
        return sensor;
    }

    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    public boolean isReversed() {
        return reversed;
    }

    public boolean onPass() {
        if (get() && !onPassReset) {
            onPassReset = true;
            return true;
        } else if (!get()) {
            onPassReset = false;
        }

        return false;
    }

    public int getCount() {
        if (get() && !onPassReset) {
            count++;
        }

        return count;
    }

    public void resetCount() {
        this.count = 0;
    }

    public boolean get() {
        boolean input = sensor.get();
        if (reversed) input = !input;

        return input;
    }
}
