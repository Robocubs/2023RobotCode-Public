package com.team1701.frc2023.subsystems;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.field.CustomAprilTagFields;
import com.team1701.lib.drivers.Limelight;
import com.team1701.lib.drivers.Limelight.LedMode;
import com.team1701.lib.util.LoggingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends Subsystem {

    public static Vision mInstance = null;
    private final LoggedDashboardChooser<FieldChoice> mFieldChooser =
            new LoggedDashboardChooser<>("Field Vision Choice");
    private VisionCamera mFrontCamera;
    private VisionCamera mBackCamera;
    private Limelight mLimelight;
    private ArrayList<VisionCamera> mCameras = new ArrayList<VisionCamera>();
    private FieldChoice mFieldSelected = null;
    private Alliance mAlliance = Alliance.Invalid;
    private AprilTagFieldLayout mAprilTagFieldLayout;
    private boolean mLoadedTagFieldLayout = false;

    public static synchronized Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }

        return mInstance;
    }

    private Vision() {
        // field options for vision
        mFieldChooser.addDefaultOption("Competition Field", FieldChoice.COMPETITION);
        mFieldChooser.addOption("UDJ Practice", FieldChoice.UDJ);
        mFrontCamera = new VisionCamera(
                Constants.kFrontCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToFrontCamPose,
                Constants.kCameraPoseStrategy);
        mBackCamera = new VisionCamera(
                Constants.kBackCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToBackCamPose,
                Constants.kCameraPoseStrategy);
        mCameras.add(mFrontCamera);
        mCameras.add(mBackCamera);
        mLimelight = new Limelight(Constants.kLimelightName);
        mLimelight.setLed(LedMode.PIPELINE);
    }

    public double getArea() {
        return mLimelight.getArea();
    }

    public double getLLXOffset() {
        return mLimelight.getXOffset();
    }

    public double getLLYOffset() {
        return mLimelight.getYOffset();
    }

    public synchronized void setLimelightMode(LimelightMode mode) {
        if (mode == LimelightMode.PIECE_DETECTION) {
            mLimelight.setPipeline(1);
        }
    }

    public synchronized LimelightMode getLimelightMode() {
        int mode = mLimelight.getPipeline();
        if (mode == 1) {
            return LimelightMode.PIECE_DETECTION;
        } else {
            return LimelightMode.UNKNOWN;
        }
    }

    public synchronized String getClassID() {
        return mLimelight.getClassID();
    }

    public synchronized boolean limelightSeesTarget() {
        return mLimelight.seesTarget();
    }

    public synchronized void setLimelightLEDState(LedMode mode) {
        mLimelight.setLed(mode);
    }

    private static AprilTagFieldLayout loadFromFile(String resourcePath) throws IOException {
        try (FileInputStream file = new FileInputStream(resourcePath)) {
            return new ObjectMapper().readerFor(AprilTagFieldLayout.class).readValue(file);
        }
    }

    public synchronized void updateAprilTagFieldLayout() throws IOException {
        var superstructure = Superstructure.getInstance();
        var newField = DriverStation.isFMSAttached() ? FieldChoice.COMPETITION : mFieldChooser.get();
        var alliance = superstructure.getAlliance();

        if (newField != mFieldSelected || (newField == FieldChoice.UDJ && alliance != mAlliance)) {
            switch (newField) {
                case COMPETITION:
                    mAprilTagFieldLayout =
                            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
                    break;
                case UDJ:
                    var file = alliance == Alliance.Blue
                            ? CustomAprilTagFields.kBlueFieldUDJ.m_resourceFile
                            : CustomAprilTagFields.kRedFieldUDJ.m_resourceFile;
                    mAprilTagFieldLayout = loadFromFile(file);
                    break;
                default:
                    mAprilTagFieldLayout =
                            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
                    break;
            }

            mCameras.forEach(c -> c.setNewAprilTagFieldLayout(mAprilTagFieldLayout));

            mLoadedTagFieldLayout = true;
            mFieldSelected = newField;
            mAlliance = alliance;

            System.out.println("Loaded new field layout");
        }
    }

    public AprilTagFieldLayout getLoadedLayout() {
        return mAprilTagFieldLayout;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            PhotonPoseEstimator poseEstimator, PhotonPipelineResult pipelineResult) {
        if (!mLoadedTagFieldLayout) return Optional.empty();
        poseEstimator.setReferencePose(PoseEstimator.getInstance().getCurrentPose());
        return poseEstimator.update(pipelineResult);
    }

    @Override
    public void readPeriodicInputs() {
        mLimelight.updateInputs();
        mLimelight.updateOutputs();
        if (!mLoadedTagFieldLayout) return;
        mCameras.forEach(c -> c.addVisionMeasurementWithConstraints(PoseEstimator.getInstance()));
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mCameras.forEach(VisionCamera::outputTelemetry);
    }

    private static enum FieldChoice {
        COMPETITION,
        UDJ
    }

    public static enum LimelightMode {
        PIECE_DETECTION,
        UNKNOWN
    }

    private class VisionCamera {
        private PhotonCamera mCamera;
        private PhotonPoseEstimator mPhotonPoseEstimator;
        private PhotonPipelineResult mPipelineResult = new PhotonPipelineResult();
        public double mPreviousCamPipelineTimestamp = 0;
        private double mLastOutputTimestampSeconds = 0;
        private String mCameraName;
        private Pose2d mRobotVisionPose = new Pose2d();
        private CameraInputs mCameraInputs;

        public VisionCamera(
                String cameraName,
                AprilTagFieldLayout initialFieldLayout,
                Transform3d robotToCamPose,
                PoseStrategy poseStrategy) {
            mCameraName = cameraName;
            mCameraInputs = new CameraInputs(new PhotonCamera(cameraName));
            mPhotonPoseEstimator = new PhotonPoseEstimator(initialFieldLayout, poseStrategy, mCamera, robotToCamPose);
            mPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        }

        public void addVisionMeasurementWithConstraints(PoseEstimator poseEstimator) {
            var estimatedCamGlobalPose =
                    Vision.getInstance().getEstimatedGlobalPose(getPhotonPoseEstimator(), getLatestPipelineResult());
            var currentTimestamp = getTimestampSeconds();
            if (currentTimestamp != mPreviousCamPipelineTimestamp && hasTargets()) {
                mPreviousCamPipelineTimestamp = currentTimestamp;
                try {
                    if (estimatedCamGlobalPose.isPresent()) {
                        mRobotVisionPose =
                                estimatedCamGlobalPose.get().estimatedPose.toPose2d();
                        if (mRobotVisionPose.getX() < 4 || mRobotVisionPose.getX() > 12.5) {
                            poseEstimator.addVisionMeasurement(mRobotVisionPose, currentTimestamp);
                        }
                    }
                } catch (ConcurrentModificationException e) {
                    return;
                }
            }
        }

        public PhotonPoseEstimator getPhotonPoseEstimator() {
            return mPhotonPoseEstimator;
        }

        public void setNewAprilTagFieldLayout(AprilTagFieldLayout fieldLayout) {
            mPhotonPoseEstimator.setFieldTags(fieldLayout);
        }

        public boolean hasTargets() {
            return mPipelineResult.hasTargets();
        }

        public double getTimestampSeconds() {
            return mPipelineResult.getTimestampSeconds();
        }

        public PhotonPipelineResult getLatestPipelineResult() {
            Logger.getInstance().processInputs("Camera/" + mCameraName, mCameraInputs);
            var photonPipelineResult = mCameraInputs.pipelineResult;

            if (!photonPipelineResult.hasTargets()) return photonPipelineResult;

            var filteredTargets = photonPipelineResult.getTargets().stream()
                    .filter(target -> target.getPoseAmbiguity() <= Constants.kCameraMaxPoseAmbiguity)
                    .toList();
            var filteredPipelineResult =
                    new PhotonPipelineResult(photonPipelineResult.getLatencyMillis(), filteredTargets);
            filteredPipelineResult.setTimestampSeconds(photonPipelineResult.getTimestampSeconds());
            mPipelineResult = filteredPipelineResult;
            return filteredPipelineResult;
        }

        public void outputTelemetry() {
            var logger = Logger.getInstance();
            var cameraNamespace = "Camera/" + mCameraName;
            if (getTimestampSeconds() != mLastOutputTimestampSeconds) {
                logger.recordOutput(cameraNamespace + "/SeesAprilTag", hasTargets());
                logger.recordOutput(cameraNamespace + "/RobotPose", mRobotVisionPose);

                mLastOutputTimestampSeconds = getTimestampSeconds();
                List<Pose3d> trackedTagPositions = new ArrayList<Pose3d>();
                for (PhotonTrackedTarget target : mPipelineResult.targets) {
                    var fiducialID = target.getFiducialId();
                    if (fiducialID != -1) {
                        trackedTagPositions.add(
                                getLoadedLayout().getTagPose(fiducialID).get());
                    }
                }

                logger.recordOutput(
                        cameraNamespace + "/AprilTagPoses",
                        trackedTagPositions.toArray(new Pose3d[trackedTagPositions.size()]));
            }
        }
    }

    private class CameraInputs implements LoggableInputs {
        private PhotonCamera mCamera;
        private PhotonPipelineResult pipelineResult;
        private boolean isConnected;

        public CameraInputs(PhotonCamera camera) {
            mCamera = camera;
        }

        @Override
        public void toLog(LogTable table) {
            isConnected = mCamera.isConnected();
            table.put("IsConnected", isConnected);

            pipelineResult = mCamera.getLatestResult();
            var targets = pipelineResult.targets;
            var targetCount = targets.size();

            table.put("Timestamp", pipelineResult.getTimestampSeconds());
            table.put("Latency", pipelineResult.getLatencyMillis());
            table.put("TargetCount", targets.size());

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";
                var target = targets.get(i);

                table.put(targetNamespace + "Yaw", target.getYaw());
                table.put(targetNamespace + "Pitch", target.getPitch());
                table.put(targetNamespace + "Skew", target.getSkew());
                table.put(targetNamespace + "Area", target.getArea());
                table.put(targetNamespace + "FiducialID", target.getFiducialId());
                table.put(targetNamespace + "PoseAmbiguity", target.getPoseAmbiguity());

                LoggingUtil.put(table, targetNamespace + "Pose", target.getBestCameraToTarget());
                LoggingUtil.put(table, targetNamespace + "AltPose", target.getAlternateCameraToTarget());

                var minAreaRectCorners = target.getMinAreaRectCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "MinAreaRectCorners", minAreaRectCorners);

                var detectedCorners = target.getDetectedCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "DetectedCorners", detectedCorners);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            var timestamp = table.getDouble("Timestamp", 0);
            var latency = (int) table.getInteger("Latency", 0);
            var targetCount = (int) table.getInteger("TargetCount", 0);
            var targets = new ArrayList<PhotonTrackedTarget>(targetCount);

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";

                var minAreaRectCornerCords =
                        table.getDoubleArray(targetNamespace + "MinAreaRectCorners", new double[0]);
                var minAreaRectCorners = new ArrayList<TargetCorner>(4);
                for (int j = 0; j < minAreaRectCornerCords.length / 2; j++) {
                    minAreaRectCorners.add(
                            j, new TargetCorner(minAreaRectCornerCords[j * 2], minAreaRectCornerCords[j * 2 + 1]));
                }

                var detectedCornerCords = table.getDoubleArray(targetNamespace + "DetectedCorners", new double[0]);
                var detectedCorners = new ArrayList<TargetCorner>(detectedCornerCords.length);
                for (int j = 0; j < detectedCornerCords.length / 2; j++) {
                    detectedCorners.add(new TargetCorner(detectedCornerCords[j * 2], detectedCornerCords[j * 2 + 1]));
                }

                var legacyFiducialID = table.getInteger(targetNamespace + "Fiducial ID", 0);

                var trackedTarget = new PhotonTrackedTarget(
                        table.getDouble(targetNamespace + "Yaw", 0),
                        table.getDouble(targetNamespace + "Pitch", 0),
                        table.getDouble(targetNamespace + "Area", 0),
                        table.getDouble(targetNamespace + "Skew", 0),
                        (int) table.getInteger(targetNamespace + "FiducialID", legacyFiducialID),
                        LoggingUtil.getTransform3d(table, targetNamespace + "Pose"),
                        LoggingUtil.getTransform3d(table, targetNamespace + "AltPose"),
                        table.getDouble(targetNamespace + "PoseAmbiguity", 0),
                        minAreaRectCorners,
                        detectedCorners);

                targets.add(trackedTarget);
            }

            pipelineResult = new PhotonPipelineResult(latency, targets);
            pipelineResult.setTimestampSeconds(timestamp);

            isConnected = table.getBoolean("IsConnected", false);
        }
    }
}
