package com.team1701.lib.drivers;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight {

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    private NetworkTable mNetworkTable;
    private String mName;
    private LimelightInputsAutoLogged mInputs = new LimelightInputsAutoLogged();
    private LimelightOutputs mOutputs = new LimelightOutputs();
    private boolean mOutputsHaveChanged = true;

    public Limelight(String tableName) {
        mName = tableName;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(tableName);
    }

    @AutoLog
    public static class LimelightInputs {
        // INPUTS
        public double latency;
        public long givenLedMode;
        public long givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public String classID = "";
        public boolean seesTarget;
    }

    public static class LimelightOutputs {
        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    public double getArea() {
        return mInputs.area;
    }

    public double getXOffset() {
        return mInputs.xOffset;
    }

    public double getYOffset() {
        return mInputs.yOffset;
    }

    public void updateInputs() {
        // Inputs
        mInputs.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + kImageCaptureLatency;
        mInputs.givenLedMode = (long) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mInputs.givenPipeline = (long) mNetworkTable.getEntry("pipeline").getDouble(0);
        mInputs.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mInputs.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mInputs.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mInputs.classID = mNetworkTable.getEntry("tclass").getString("none");
        mInputs.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        Logger.getInstance().processInputs("Limelight/" + mName, mInputs);
    }

    public void updateOutputs() {
        // Outputs
        if (mInputs.givenLedMode != mOutputs.ledMode || mInputs.givenPipeline != mOutputs.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mOutputs.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mOutputs.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mOutputs.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mOutputs.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mOutputs.snapshot);
            mOutputsHaveChanged = false;
        }
    }

    public void setCamMode(int i) {
        if (i != mOutputs.camMode) {
            mOutputs.camMode = i;
            mOutputsHaveChanged = true;
        }
    }

    public enum LedMode {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    public void setLed(LedMode mode) {
        if (mode.ordinal() != mOutputs.ledMode) {
            mOutputs.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public void setPipeline(int mode) {
        mOutputs.pipeline = mode;
        mOutputsHaveChanged = true;
    }

    public void setStream(int stream) {
        if (stream != mOutputs.stream) {
            mOutputs.stream = stream;
            mOutputsHaveChanged = true;
        }
    }

    public void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public int getPipeline() {
        return mOutputs.pipeline;
    }

    public String getClassID() {
        return mInputs.classID;
    }

    public boolean seesTarget() {
        return mInputs.seesTarget;
    }

    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::getY);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        var corners = new ArrayList<Translation2d>();
        for (var i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(ySort);

        var leftCorner = corners.get(0);
        var rightCorner = corners.get(1);

        return List.of(
                new double[] {leftCorner.getX(), leftCorner.getY()},
                new double[] {rightCorner.getX(), rightCorner.getY()});
    }

    public double getLatency() {
        return mInputs.latency;
    }
}
