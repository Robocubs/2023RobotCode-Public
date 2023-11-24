package com.team1701.lib.drivers;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class TalonSRXFactory {

    private static final int kTimeoutMs = 100;

    public static class Configuration extends TalonSRXConfiguration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public Configuration() {
            closedloopRamp = 0.05;
        }
    }

    public static TalonSRX createTalon(int id, Consumer<Configuration> config) {
        var configuration = new Configuration();
        config.accept(configuration);
        return createTalon(id, configuration);
    }

    public static TalonSRX createTalon(int id, Configuration config) {
        TalonSRX talon = new LazyTalonSRX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.clearStickyFaults(kTimeoutMs);

        talon.configAllSettings(config, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.enableVoltageCompensation(false);

        return talon;
    }
}
