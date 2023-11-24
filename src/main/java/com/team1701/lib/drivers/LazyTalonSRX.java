package com.team1701.lib.drivers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonSRX extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected TalonSRXControlMode mLastControlMode = null;

    public LazyTalonSRX(int i) {
        super(i);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(TalonSRXControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}
