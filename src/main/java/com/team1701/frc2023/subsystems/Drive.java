// The core logic in this code was derived from Team 254's 2022 robot code
// https://github.com/Team254/FRC-2022-Public
// THE REPOSITORY RELEASED WITH THIS CODE WAS/IS UNDER THE MIT LICENSE
package com.team1701.frc2023.subsystems;

import java.util.List;
import java.util.stream.Stream;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.lib.swerve.SwerveSetpoint;
import com.team1701.lib.swerve.SwerveSetpointGenerator;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.swerve.SwerveTrajectoryController;
import com.team1701.lib.swerve.SwerveWaypointController;
import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.LoggingUtil;
import com.team1701.lib.util.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Drive extends Subsystem {
    private static Drive mInstance = null;
    private final DrivePeriodicInputs mPeriodicInputs = new DrivePeriodicInputs();
    private final DrivePeriodicOutputs mDrivePeriodicOutputs = new DrivePeriodicOutputs();

    private final Pigeon2 mPigeon = new Pigeon2(Constants.kPigeonIMUId);

    private final FalconSwerveModule[] mModules;
    public static final int kModuleCount = Constants.kKinematics.getNumModules();
    public static final int kFrontLeftModuleIdx = 0;
    public static final int kFrontRightModuleIdx = 1;
    public static final int kBackLeftModuleIdx = 2;
    public static final int kBackRightModuleIdx = 3;

    private final SwerveSetpointGenerator mSetpointGenerator;
    private final SwerveTrajectoryController mTrajectoryController;
    private final SwerveWaypointController mWaypointController;
    private final PIDController mAutoBalanceRotationController;

    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    private KinematicLimits mKinematicLimits = Constants.kSmoothKinematicLimits;

    private double mYawOffset = 0;
    private boolean mAutoBalanceCanReverse = false;

    DriveValuesLogged mLoggedValues = new DriveValuesLogged();

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive() {
        mModules = new FalconSwerveModule[kModuleCount];

        /*
         * Creating each module
         *
         * drive Talon ID
         * steering Talon ID
         * Cancoder instance
         * Some hard offset to correct CANCoder
         */
        mModules[kFrontLeftModuleIdx] = new FalconSwerveModule(
                Constants.kFrontLeftDriveTalonId,
                Constants.kFrontLeftAziTalonId,
                Cancoders.getInstance().getFrontLeft(),
                Constants.kFrontLeftAziEncoderOffset);

        mModules[kFrontRightModuleIdx] = new FalconSwerveModule(
                Constants.kFrontRightDriveTalonId,
                Constants.kFrontRightAziTalonId,
                Cancoders.getInstance().getFrontRight(),
                Constants.kFrontRightAziEncoderOffset);

        mModules[kBackLeftModuleIdx] = new FalconSwerveModule(
                Constants.kBackLeftDriveTalonId,
                Constants.kBackLeftAziTalonId,
                Cancoders.getInstance().getBackLeft(),
                Constants.kBackLeftAziEncoderOffset);

        mModules[kBackRightModuleIdx] = new FalconSwerveModule(
                Constants.kBackRightDriveTalonId,
                Constants.kBackRightAziTalonId,
                Cancoders.getInstance().getBackRight(),
                Constants.kBackRightAziEncoderOffset);

        mPigeon.configSetParameter(ParamEnum.eConfigMountPoseYaw, 90.0, 0, 0, 100);
        mPigeon.configSetParameter(ParamEnum.eConfigMountPoseRoll, -0.5, 0, 0, 100);

        readGyro(true);
        readModules();
        setSetpointFromMeasured();

        // How often the sensor updates internally
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        mSetpointGenerator = new SwerveSetpointGenerator(Constants.kKinematics);

        mWaypointController = new SwerveWaypointController(
                new PIDController(2, 0, 0, Constants.kLooperDt),
                new PIDController(4, 0, 0, Constants.kLooperDt),
                new TrapezoidProfile.Constraints(3.5, 2.5),
                Constants.kLooperDt,
                0.01,
                Rotation2d.fromDegrees(0.5));

        mAutoBalanceRotationController = new PIDController(4, 0, 0, Constants.kLooperDt);
        mAutoBalanceRotationController.enableContinuousInput(-Math.PI, Math.PI);
        mAutoBalanceRotationController.setTolerance(0.02);

        mTrajectoryController = new SwerveTrajectoryController(
                new PIDController(4, 0, 0, Constants.kLooperDt),
                new PIDController(2, 0, 0, Constants.kLooperDt),
                new PIDController(2, 0, 0, Constants.kLooperDt));

        resetAzimuth();
    }

    public class DriveValuesLogged implements LoggableInputs {
        @Override
        public void toLog(LogTable table) {
            for (int i = 0; i < mModules.length; i++) {
                var swerveModuleNamespace = "SwerveModule/" + i + "/";
                table.put(swerveModuleNamespace + "Speed", mPeriodicInputs.measuredStates[i].speedMetersPerSecond);
                table.put(swerveModuleNamespace + "Distance", mPeriodicInputs.measuredPositions[i].distanceMeters);
                LoggingUtil.put(table, swerveModuleNamespace + "Rotation", mPeriodicInputs.measuredStates[i].angle);
                LoggingUtil.put(table, swerveModuleNamespace + "AbsoluteRotation", mPeriodicInputs.absoluteAngles[i]);
            }

            table.put("Gyro/Yaw", mPeriodicInputs.rawYaw);
            table.put("Gyro/Pitch", mPeriodicInputs.rawPitch);
            table.put("Gyro/Roll", mPeriodicInputs.rawRoll);
        }

        @Override
        public void fromLog(LogTable table) {
            for (int i = 0; i < kModuleCount; i++) {
                var swerveModuleNamespace = "SwerveModule/" + i + "/";

                var rotation = LoggingUtil.getRotation2d(table, swerveModuleNamespace + "Rotation");
                mPeriodicInputs.measuredStates[i] =
                        new SwerveModuleState(table.getDouble(swerveModuleNamespace + "Speed", 0), rotation);
                mPeriodicInputs.measuredPositions[i] =
                        new SwerveModulePosition(table.getDouble(swerveModuleNamespace + "Distance", 0), rotation);
                mPeriodicInputs.absoluteAngles[i] =
                        LoggingUtil.getRotation2d(table, swerveModuleNamespace + "AbsoluteRotation");
            }

            mPeriodicInputs.rawYaw = table.getDouble("Gyro/Yaw", 0);
            mPeriodicInputs.rawPitch = table.getDouble("Gyro/Pitch", 0);
            mPeriodicInputs.rawRoll = table.getDouble("Gyro/Roll", 0);

            readGyro(false);
        }
    }

    public static class DrivePeriodicInputs {
        ChassisSpeeds desChassisSpeeds = new ChassisSpeeds();

        SwerveModuleState[] measuredStates =
                Stream.generate(SwerveModuleState::new).limit(kModuleCount).toArray(SwerveModuleState[]::new);
        SwerveModulePosition[] measuredPositions =
                Stream.generate(SwerveModulePosition::new).limit(kModuleCount).toArray(SwerveModulePosition[]::new);
        Rotation2d[] absoluteAngles =
                Stream.generate(Rotation2d::new).limit(kModuleCount).toArray(Rotation2d[]::new);

        double rawYaw;
        double rawPitch;
        double rawRoll;
        Rotation3d heading = new Rotation3d();
    }

    public static class DrivePeriodicOutputs {
        SwerveSetpoint setpoint =
                new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[Constants.kKinematics.getNumModules()]);

        boolean want_orient = false;
    }

    public synchronized SwerveModulePosition[] getCurrentSwerveModulePositions() {
        return mPeriodicInputs.measuredPositions;
    }

    /*
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public synchronized void zeroGyroscope() {
        zeroGyroscope(0);
    }

    public synchronized void zeroGyroscope(double yaw) {
        var poseEstimator = PoseEstimator.getInstance();
        var initialPose = poseEstimator.getCurrentPose();

        mYawOffset = mPeriodicInputs.rawYaw - yaw;
        readGyro(false);

        // If our gyro offset changes, PoseEstimator will think we turned. Call setPose with the initial pose to make
        // PoseEstimator take a new gyro reading without changing the pose.
        poseEstimator.setPose(initialPose);
    }

    public synchronized void zeroGyroscope(Rotation2d rotation) {
        var normalizedDegrees = MathUtil.inputModulus(rotation.getDegrees(), -180, 180);
        zeroGyroscope(normalizedDegrees);
    }

    public synchronized Rotation2d getFieldRelativeGyroscopeRotation() {
        return mPeriodicInputs.heading.toRotation2d();
    }

    protected synchronized void readGyro(boolean updateRawValues) {
        if (updateRawValues) {
            mPeriodicInputs.rawYaw = mPigeon.getYaw();
            mPeriodicInputs.rawPitch = mPigeon.getPitch();
            mPeriodicInputs.rawRoll = mPigeon.getRoll();
        }

        mPeriodicInputs.heading = new Rotation3d(
                Units.degreesToRadians(mPeriodicInputs.rawRoll),
                Units.degreesToRadians(mPeriodicInputs.rawPitch),
                Units.degreesToRadians(mPeriodicInputs.rawYaw - mYawOffset));
    }

    private void setSteerCoastMode() {
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].setSteerCoastMode();
        }
    }

    private void setSteerBrakeMode() {
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].setSteerBrakeMode();
        }
    }

    /*
     * setVelocity:
     * Sets the desired chassis speed (dx, dy, and angular velocity.)
     * Makes sure that want_orient is false so modules don't stop and rotate.
     */
    public synchronized void setVelocity(ChassisSpeeds chassisSpeeds) {
        if (mDriveControlState != DriveControlState.VELOCITY_CONTROL) {
            return;
        }
        mPeriodicInputs.desChassisSpeeds = chassisSpeeds;
        mDrivePeriodicOutputs.want_orient = false;
    }

    public synchronized void setWaypoint(Pose2d targetPose) {
        if (mWaypointController.getGoal().equals(targetPose) && mDriveControlState == DriveControlState.WAYPOINT) {
            return;
        }

        setKinematicLimits(Constants.kFastKinematicLimits);
        mWaypointController.setGoal(targetPose, PoseEstimator.getInstance().getCurrentPose());
        mDriveControlState = DriveControlState.WAYPOINT;
    }

    public void setPath(PathPlannerTrajectory trajectory) {
        mDriveControlState = DriveControlState.PATH_FOLLOWING;
        setKinematicLimits(Constants.kFastKinematicLimits);
        mTrajectoryController.setTrajectory(trajectory);
        mDrivePeriodicOutputs.want_orient = false;
    }

    public synchronized boolean robotIsAtWaypoint() {
        return mWaypointController.atWaypoint();
    }

    public synchronized void setWantOrient(boolean wantOrient) {
        mDrivePeriodicOutputs.want_orient = wantOrient;
        if (!wantOrient) {
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].stop();
            }
        }
    }

    /*
     * orientModules:
     * Orients the modules without moving.
     * Most likely used in auton/startup.
     */
    public synchronized void orientModulesSwerveLock() {
        orientModules(List.of(
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(45)));
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mDrivePeriodicOutputs.setpoint.mModuleStates[i] = new SwerveModuleState(0.0, orientations.get(i));
        }
    }

    public synchronized void orientModules(Rotation2d orientation) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mDrivePeriodicOutputs.setpoint.mModuleStates[i] = new SwerveModuleState(0.0, orientation);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        readGyro(true);
        readModules();

        Logger.getInstance().processInputs("Drive", mLoggedValues);
    }

    /*
     * readModules:
     * Gets the state of each modules and upadtes the corresponding mPeriodicIO global
     * Gets the CANCoder angle in degrees and updates the corresponding mPeriodicIO global
     */
    private void readModules() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicInputs.measuredStates[i] = mModules[i].getState();
            mPeriodicInputs.measuredPositions[i] = mModules[i].getPosition();
            mPeriodicInputs.absoluteAngles[i] = mModules[i].getCanCoderAngle();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; ++i) {
            if (!mDrivePeriodicOutputs.want_orient) {
                mModules[i].setWithVelocityShortestPath(
                        mDrivePeriodicOutputs.setpoint.mModuleStates[i].speedMetersPerSecond,
                        mDrivePeriodicOutputs.setpoint.mModuleStates[i].angle);
            } else {
                mModules[i].setWithVoltageShortestPath(0.0, mDrivePeriodicOutputs.setpoint.mModuleStates[i].angle);
            }
        }
    }

    public synchronized void setSetpointFromMeasured() {
        for (int i = 0; i < mModules.length; ++i) {
            mDrivePeriodicOutputs.setpoint.mModuleStates[i] = mPeriodicInputs.measuredStates[i];
        }
        mDrivePeriodicOutputs.setpoint.mChassisSpeeds =
                Constants.kKinematics.toChassisSpeedWheelConstraints(mDrivePeriodicOutputs.setpoint.mModuleStates);
    }

    // Reconfigure periodically in case an error was thrown the first time
    public void reconfigureTalons() {
        for (FalconSwerveModule module : mModules) {
            module.configureTalons();
        }
    }

    public void setDriveControlState(DriveControlState state) {
        if (mDriveControlState == state) {
            return;
        }

        if (state == DriveControlState.AUTO_BALANCE) {
            mAutoBalanceCanReverse = false;

            var rotation = MathUtil.angleModulus(mPeriodicInputs.heading.getZ());
            var rotationSetpoint = Util.inRange(rotation, Math.PI / 2) ? 0 : Math.PI;
            mAutoBalanceRotationController.setSetpoint(rotationSetpoint);
            mAutoBalanceRotationController.reset();
        }

        mDriveControlState = state;
    }

    public enum DriveControlState {
        VELOCITY_CONTROL,
        PATH_FOLLOWING,
        WAYPOINT,
        AUTO_BALANCE,
        BRAKE
    }

    public synchronized void setKinematicLimits(KinematicLimits limits) {
        if (limits != mKinematicLimits) {
            mKinematicLimits = limits;
        }
    }

    public synchronized KinematicLimits getKinematicLimits() {
        return mKinematicLimits;
    }

    public synchronized void resetAzimuth() {
        for (FalconSwerveModule module : mModules) {
            module.rezeroSteeringMotor();
        }
        // Force a module read.
        readModules();
    }

    public void engageSwerveLock() {
        mDriveControlState = DriveControlState.BRAKE;
    }

    private void updateWaypoint() {
        if (robotIsAtWaypoint()) {
            orientModulesSwerveLock();
            return;
        }

        var pose = PoseEstimator.getInstance().getCurrentPose();
        mPeriodicInputs.desChassisSpeeds =
                mWaypointController.calculate(pose, mDrivePeriodicOutputs.setpoint.mChassisSpeeds);
    }

    private void updateBrake() {
        var chassisSpeeds = getChassisSpeeds();
        var robotSpeedSquared = chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond
                + chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond;
        if (robotSpeedSquared > Constants.kMinLockSpeed * Constants.kMinLockSpeed) {
            mPeriodicInputs.desChassisSpeeds = new ChassisSpeeds();
            mDrivePeriodicOutputs.want_orient = false;
        } else {
            orientModulesSwerveLock();
        }
    }

    private void updateAutoBalance() {
        var speed = Constants.kMinDriveSpeed * 1.5;
        var yaw = mPeriodicInputs.heading.toRotation2d();
        var rotationOutput = mAutoBalanceRotationController.calculate(yaw.getRadians());
        var pitch = Units.radiansToDegrees(mPeriodicInputs.heading.getY());
        if (pitch > 10.5) {
            mPeriodicInputs.desChassisSpeeds = new ChassisSpeeds(speed, 0, rotationOutput);
            mDrivePeriodicOutputs.want_orient = false;
            mAutoBalanceCanReverse = true;
        } else if (pitch > 6.0 && mAutoBalanceCanReverse) {
            mPeriodicInputs.desChassisSpeeds = new ChassisSpeeds(-speed, 0, rotationOutput);
            mDrivePeriodicOutputs.want_orient = false;
        } else if (pitch < -10.5) {
            mPeriodicInputs.desChassisSpeeds = new ChassisSpeeds(-speed, 0, rotationOutput);
            mDrivePeriodicOutputs.want_orient = false;
            mAutoBalanceCanReverse = true;
        } else if (pitch < -6.0 && mAutoBalanceCanReverse) {
            mPeriodicInputs.desChassisSpeeds = new ChassisSpeeds(speed, 0, rotationOutput);
            mDrivePeriodicOutputs.want_orient = false;
        } else {
            orientModulesSwerveLock();
            mAutoBalanceCanReverse = false;
        }
    }

    private void updateDesiredStates() {
        // Set the des_states to account for robot traversing arc.
        Pose2d robot_pose_vel = new Pose2d(
                mPeriodicInputs.desChassisSpeeds.vxMetersPerSecond * Constants.kLooperDt,
                mPeriodicInputs.desChassisSpeeds.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(mPeriodicInputs.desChassisSpeeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = GeometryUtil.kPoseIdentity.log(robot_pose_vel);

        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.kLooperDt,
                twist_vel.dy / Constants.kLooperDt,
                twist_vel.dtheta / Constants.kLooperDt);
        mDrivePeriodicOutputs.setpoint = mSetpointGenerator.generateSetpoint(
                mKinematicLimits, mDrivePeriodicOutputs.setpoint, updated_chassis_speeds, Constants.kLooperDt);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.kKinematics.toChassisSpeeds(mPeriodicInputs.measuredStates);
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setSetpointFromMeasured();
                setDriveControlState(DriveControlState.VELOCITY_CONTROL);
                setVelocity(new ChassisSpeeds(0, 0, 0));
                setKinematicLimits(Constants.kSmoothKinematicLimits);
                setSteerBrakeMode();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    if (mDriveControlState == DriveControlState.WAYPOINT) {
                        updateWaypoint();
                    } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
                        mPeriodicInputs.desChassisSpeeds = mTrajectoryController.update(
                                PoseEstimator.getInstance().getCurrentPose());
                    } else if (mDriveControlState == DriveControlState.BRAKE) {
                        updateBrake();
                    } else if (mDriveControlState == DriveControlState.AUTO_BALANCE) {
                        updateAutoBalance();
                    }

                    if (!mDrivePeriodicOutputs.want_orient) {
                        updateDesiredStates();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Drive.class.getSimpleName();
            }
        });
    }

    @Override
    public void stop() {
        setVelocity(new ChassisSpeeds());
        setSteerCoastMode();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        var logger = Logger.getInstance();
        logger.recordOutput("Drive/IsAtWaypoint", robotIsAtWaypoint());
        logger.recordOutput("Drive/ControlState", mDriveControlState.toString());
        LoggingUtil.recordOutput("Drive/ChassisSpeeds", mPeriodicInputs.desChassisSpeeds);
        logger.recordOutput(
                "Drive/Degrees/driveFieldRelativeDegrees", Units.radiansToDegrees(mPeriodicInputs.heading.getZ()));
        logger.recordOutput("Drive/Degrees/Pitch", Units.radiansToDegrees(mPeriodicInputs.heading.getY()));
        logger.recordOutput("Drive/Degrees/Roll", Units.radiansToDegrees(mPeriodicInputs.heading.getX()));

        logger.recordOutput("Drive/ModuleMeasuredStates", mPeriodicInputs.measuredStates);
        logger.recordOutput("Drive/RequestedWaypoint", mWaypointController.getGoal());
    }
}
