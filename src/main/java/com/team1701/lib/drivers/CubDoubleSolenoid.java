package com.team1701.lib.drivers;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CubDoubleSolenoid extends DoubleSolenoid {

    private Value lastValue = null;

    public CubDoubleSolenoid(int module, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }

    @Override
    public void set(final Value value) {
        if (lastValue != value) {
            lastValue = value;
            super.set(value);
        }
    }

    public void clearCache() {
        lastValue = null;
    }
}
