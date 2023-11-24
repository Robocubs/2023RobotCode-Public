package com.team1701.frc2023.subsystems;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Pneumatics extends Subsystem {
    private static Pneumatics mInstance = null;
    private final Compressor mCompressor = new Compressor(Constants.kPneumaticsHubId, PneumaticsModuleType.REVPH);
    private double mPressure;
    private boolean mIsHackyCompression;
    private final PneumaticsInputsAutoLogged mPeriodicInputs = new PneumaticsInputsAutoLogged();

    @AutoLog
    public static class PneumaticsInputs {
        double PSI;
    }

    public static Pneumatics getInstance() {
        if (mInstance == null) {
            mInstance = new Pneumatics();
        }
        return mInstance;
    }

    public Pneumatics() {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                startCompressor();
            }

            @Override
            public void onLoop(double timestamp) {
                if (mIsHackyCompression && mCompressor.isEnabled()) {
                    startCompressor();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stopCompressor();
            }

            @Override
            public String getDisplayName() {
                return Pneumatics.class.getSimpleName();
            }
        });
    }

    private void startCompressor() {
        mCompressor.enableAnalog(Constants.kCompressorMinPressure, Constants.kCompressorMaxPressure);
        mIsHackyCompression = false;
    }

    public void doInstantHackyCompression() {
        var minPressure = Constants.kCompressorMaxPressure - 5;
        if (mPressure > minPressure) {
            mCompressor.enableAnalog(minPressure, Constants.kCompressorMaxPressure);
            mIsHackyCompression = true;
        }
    }

    private void stopCompressor() {
        mCompressor.disable();
    }

    @Override
    public void stop() {
        stopCompressor();
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void readPeriodicInputs() {
        mPressure = mCompressor.getPressure();
        mPeriodicInputs.PSI = mPressure;
        Logger.getInstance().processInputs("Pneumatics", mPeriodicInputs);
    }

    @Override
    public void outputTelemetry() {}
}
