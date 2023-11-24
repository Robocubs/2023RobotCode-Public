package com.team1701.frc2023.subsystems;

import java.util.Optional;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.subsystems.Vision.LimelightMode;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class PieceSeeker extends Subsystem {
    private static PieceSeeker mPieceLockController = null;
    private PoseEstimator mPoseEstimator;
    private Vision mVision;

    private Optional<Pose2d> mPiecePosition = Optional.empty();
    private Pose2d mLastPiecePosition = new Pose2d();
    private double mLastCalculatedDistance = 0.0;
    private final double kLastLockTimeout = 3.5;
    private double kMaxDistance = Constants.kMaxAutonDistance;
    private PieceFilterStrategy mPieceFilterStrategy = PieceFilterStrategy.RADIUS_FROM_EXPECTED;
    private double mLastLockTimestamp = 0.0;
    private Translation2d mExpectedPosition = new Translation2d();
    private boolean mPassesFilter = false;

    public static PieceSeeker getInstance() {
        if (mPieceLockController == null) {
            mPieceLockController = new PieceSeeker();
        }
        return mPieceLockController;
    }

    public void setExpectedPosition(Translation2d position) {
        mExpectedPosition = position;
    }

    public void setPieceFilterStrategy(PieceFilterStrategy strat, double distance) {
        mPieceFilterStrategy = strat;
        kMaxDistance = distance;
    }

    public boolean getPieceFilterPass() {
        return mPassesFilter;
    }

    private boolean passesPieceFilter(Pose2d lineupPose, Pose2d currentPose) {
        switch (mPieceFilterStrategy) {
            case DISTANCE_TO_PIECE:
                return lineupPose.getTranslation().getDistance(currentPose.getTranslation()) <= kMaxDistance;
            case RADIUS_FROM_EXPECTED:
                if (mExpectedPosition == null) {
                    return false;
                }
                return mExpectedPosition.getDistance(lineupPose.getTranslation()) <= kMaxDistance;
            default:
                return false;
        }
    }

    public PieceSeeker() {
        mPoseEstimator = PoseEstimator.getInstance();
        mVision = Vision.getInstance();
    }

    /**
     * Uses Limelight values to calculate piece position. Should be called every loop
     */
    private synchronized void updateTargetInputs() {
        if (mVision.getLimelightMode() != LimelightMode.PIECE_DETECTION) {
            mPiecePosition = Optional.empty();
            return;
        }
        double currTime = Timer.getFPGATimestamp();
        if (!mVision.limelightSeesTarget()) {
            if (currTime - mLastLockTimestamp > kLastLockTimeout) {
                mPiecePosition = Optional.empty();
            }
            return;
        }
        mPiecePosition = getLimelightPiecePosition(
                mPoseEstimator.getCurrentPose(), mVision.getLLXOffset(), mVision.getLLYOffset());

        mLastLockTimestamp = currTime;
    }

    public synchronized Optional<Pose2d> getPiecePosition() {
        return mPiecePosition;
    }

    private synchronized Translation2d getLimelightTranslationToPiece(double tx, double ty) {
        double x = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.kRobotToLimelightPose.getZ(),
                Constants.kCubeGamePieceHeight / 1.5,
                Constants.kRobotToLimelightPose.getRotation().getY(),
                Math.toRadians(ty));
        double y = x * Math.tan(Math.toRadians(tx));

        return new Translation2d(x, -y);
    }

    private synchronized Optional<Pose2d> getLimelightPiecePosition(Pose2d currentRobotPose, double tx, double ty) {
        Translation2d t = getLimelightTranslationToPiece(tx, ty);

        mLastCalculatedDistance = t.getNorm();

        var limelightTransform2d = new Transform2d(
                new Translation2d(Constants.kRobotToLimelightPose.getX(), Constants.kRobotToLimelightPose.getY()),
                new Rotation2d(Constants.kRobotToLimelightPose.getRotation().getZ()));

        var piecePoseNoRotation =
                currentRobotPose.transformBy(limelightTransform2d).transformBy(new Transform2d(t, new Rotation2d()));

        var pieceLineupRotation = piecePoseNoRotation
                .getTranslation()
                .minus(currentRobotPose.getTranslation())
                .getAngle();

        var pieceLineupPose = new Pose2d(piecePoseNoRotation.getTranslation(), pieceLineupRotation)
                // Add arbitrary offsets here
                .transformBy(new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()));

        mLastPiecePosition = pieceLineupPose;

        if (passesPieceFilter(pieceLineupPose, currentRobotPose)) {
            mPassesFilter = true;
            return Optional.of(pieceLineupPose);
        } else {
            mPassesFilter = false;
            return Optional.empty();
        }
    }

    public enum PieceFilterStrategy {
        DISTANCE_TO_PIECE,
        RADIUS_FROM_EXPECTED
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        var logger = Logger.getInstance();
        if (!mPiecePosition.isEmpty()) {
            logger.recordOutput("PieceSeeker/Pose", mPiecePosition.get());
        }

        logger.recordOutput("PieceSeeker/LastPose", mLastPiecePosition);
        logger.recordOutput("PieceSeeker/FilterStrategy", mPieceFilterStrategy.toString());
        logger.recordOutput("PieceSeeker/ExpectedPose", new Pose2d(mExpectedPosition, GeometryUtil.kRotationIdentity));
        logger.recordOutput("PieceSeeker/TrackedPiece/PassesFilter", mPassesFilter);
        logger.recordOutput("PieceSeeker/TrackedPiece/Distance", mLastCalculatedDistance);
    }

    @Override
    public void readPeriodicInputs() {
        updateTargetInputs();
    }
}
