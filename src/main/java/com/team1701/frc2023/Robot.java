// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1701.frc2023;

import java.io.IOException;

import com.team1701.frc2023.autonomous.AutonomousChooser;
import com.team1701.frc2023.autonomous.AutonomousMode;
import com.team1701.frc2023.autonomous.AutonomousModes;
import com.team1701.frc2023.loops.Looper;
import com.team1701.frc2023.subsystems.Arm;
import com.team1701.frc2023.subsystems.Arm.ArmControlState;
import com.team1701.frc2023.subsystems.Brakes;
import com.team1701.frc2023.subsystems.Cancoders;
import com.team1701.frc2023.subsystems.Drive;
import com.team1701.frc2023.subsystems.Drive.DriveControlState;
import com.team1701.frc2023.subsystems.Hand;
import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake;
import com.team1701.frc2023.subsystems.LED;
import com.team1701.frc2023.subsystems.OperatorInterfaceClient;
import com.team1701.frc2023.subsystems.PieceSeeker;
import com.team1701.frc2023.subsystems.PieceSeeker.PieceFilterStrategy;
import com.team1701.frc2023.subsystems.Pneumatics;
import com.team1701.frc2023.subsystems.PoseEstimator;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;
import com.team1701.frc2023.subsystems.Vision;
import com.team1701.frc2023.subsystems.Vision.LimelightMode;
import com.team1701.lib.util.CrashTracker;
import com.team1701.lib.util.LEDState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private final Looper mEnabledLooper = new Looper("enabled");
    private final Looper mDisabledLooper = new Looper("disabled");

    private final AutonomousChooser mAutonomousChooser = AutonomousChooser.getInstance();
    private AutonomousMode mAutonomous = AutonomousModes.none();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final ControllerManager mControllerManager = ControllerManager.getInstance();
    private final Cancoders mCancoders = Cancoders.getInstance();
    private final Vision mVision = Vision.getInstance();
    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
    private final Hand mHand = Hand.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Pneumatics mPneumatics = Pneumatics.getInstance();
    private final Brakes mBrakes = Brakes.getInstance();
    private final OperatorInterfaceClient mOperatorInterfaceClient = OperatorInterfaceClient.getInstance();
    private final LED mLED = LED.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final PieceSeeker mPieceSeeker = PieceSeeker.getInstance();

    public Robot() {
        super();

        addPeriodic(
                () -> {
                    mEnabledLooper.loop();
                    mDisabledLooper.loop();
                },
                Constants.kLooperDt,
                0.005);
    }

    private void createJoystickHandlers() {
        var driverJoystick = mControllerManager.getDriverJoystick();
        var operatorJoystick = mControllerManager.getOperatorJoystick();

        driverJoystick.onButtonPressed(ControllerManager.kXBOXButtonX, () -> {
            var yaw = mSuperstructure.getAlliance() == Alliance.Blue ? 0 : 180;
            mDrive.zeroGyroscope(yaw);
        });
        driverJoystick.onAxisPressed(Constants.kDriverTurboBoostBumper, () -> {
            mDrive.setKinematicLimits(Constants.kFastKinematicLimits);
        });
        driverJoystick.onAxisReleased(Constants.kDriverTurboBoostBumper, () -> {
            mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
        });
        driverJoystick.onAxisPressed(Constants.kDriverSwerveLockButton, () -> {
            mSuperstructure.engageSwerveLock();
        });
        driverJoystick.onAxisReleased(Constants.kDriverSwerveLockButton, () -> {
            mSuperstructure.disengageSwerveLock();
        });
        driverJoystick.onButtonPressed(Constants.kDriverAllowOperatorAction, () -> {
            mSuperstructure.startAutoPosition();
        });
        driverJoystick.onButtonReleased(Constants.kDriverAllowOperatorAction, () -> {
            mSuperstructure.stopAutoPosition();
        });

        driverJoystick.onButtonPressed(ControllerManager.kXBOXButtonA, () -> {
            var pos = mPieceSeeker.getPiecePosition();
            if (!pos.isEmpty()) {
                mDrive.setWaypoint(pos.get());
            }
        });

        driverJoystick.onButtonReleased(ControllerManager.kXBOXButtonA, () -> {
            mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        });

        // If we strafe, throttle, or rotate, we should set DriveControlState to VELOCITY_CONTROL
        driverJoystick.onXPressed(() -> {
            mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        });
        driverJoystick.onYPressed(() -> {
            mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        });
        driverJoystick.onZPressed(() -> {
            mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        });

        driverJoystick.onButtonPressed(ControllerManager.kXBOXButtonB, () -> {
            mDrive.setDriveControlState(DriveControlState.AUTO_BALANCE);
        });

        driverJoystick.onButtonPressed(Constants.kDriverSlowModeButton, () -> {
            mDrive.setKinematicLimits(Constants.kSlowModeKinematicLimits);
        });
        driverJoystick.onButtonReleased(Constants.kDriverSlowModeButton, () -> {
            mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
        });

        operatorJoystick.onXPressed(() -> {
            mSuperstructure.setTargetType(TargetType.NONE);
        });
        operatorJoystick.onYPressed(() -> {
            mSuperstructure.setTargetType(TargetType.NONE);
        });

        // *** Manual operator controls MUST interrupt a handoff IF it is happening ***
        operatorJoystick.onButtonPressed(ControllerManager.kXBOXBumperLeft, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mSuperstructure.setTargetType(TargetType.HOLD);
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXBumperLeft, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });

        operatorJoystick.onButtonPressed(ControllerManager.kXBOXBumperRight, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mHand.toggleHandState();
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXBumperRight, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });

        operatorJoystick.onButtonPressed(ControllerManager.kXBOXButtonX, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mSuperstructure.setTargetType(TargetType.SUBSTATION);
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXButtonX, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });

        operatorJoystick.onButtonPressed(ControllerManager.kXBOXButtonY, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mSuperstructure.setTargetType(TargetType.HIGH_CONE);
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXButtonY, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });

        operatorJoystick.onButtonPressed(ControllerManager.kXBOXButtonB, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mSuperstructure.setTargetType(TargetType.MID_CONE);
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXButtonB, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });

        operatorJoystick.onButtonPressed(ControllerManager.kXBOXButtonA, () -> {
            mSuperstructure.interruptHandoffStateMachine();
            mSuperstructure.setTargetType(TargetType.LOW);
        });
        operatorJoystick.onButtonReleased(ControllerManager.kXBOXButtonA, () -> {
            mSuperstructure.restartHandoffStateMachine();
        });
        // *** Manual operator controls MUST interrupt a handoff IF it is happening ***
    }

    private void initializeAdvantageKit() {
        var logger = Logger.getInstance();

        // Record metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        if (isReal()) {
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            logger.setReplaySource(new WPILOGReader(logPath));
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.getInstance().disableDeterministicTimestamps()

        // Start AdvantageKit logger
        logger.start();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            initializeAdvantageKit();

            // Subsystem loops are executed in order of registration.
            mSubsystemManager.setSubsystems(
                    mLED,
                    mCancoders,
                    mDrive,
                    mVision,
                    mPoseEstimator,
                    mBrakes,
                    mPneumatics,
                    mArm,
                    mHand,
                    mIntake,
                    mSuperstructure,
                    mOperatorInterfaceClient,
                    mPieceSeeker);
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mSubsystemManager.outputToSmartDashboard();
            createJoystickHandlers();
            AutonomousModes.initializePaths();
            mVision.updateAprilTagFieldLayout();
            mVision.setLimelightMode(LimelightMode.PIECE_DETECTION);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mVision.updateAprilTagFieldLayout();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        mSubsystemManager.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mDisabledLooper.outputToSmartDashboard();
        mLED.pushStateToLEDControllers();
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mEnabledLooper.start();

            mDrive.setKinematicLimits(Constants.kAutonomousKinematicLimits);
            mHand.setHandState(HandState.CLOSE);
            mPieceSeeker.setPieceFilterStrategy(PieceFilterStrategy.RADIUS_FROM_EXPECTED, Constants.kMaxAutonDistance);
            mAutonomous = mAutonomousChooser.getSelectedMode();
            mAutonomous.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        mAutonomous.loop();
    }

    @Override
    public void autonomousExit() {
        mAutonomous.stop();
        mAutonomous = AutonomousModes.none();
        mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mControllerManager.resetHandlers();
            mEnabledLooper.start();

            mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
            mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);

            mPieceSeeker.setPieceFilterStrategy(PieceFilterStrategy.DISTANCE_TO_PIECE, Units.feetToMeters(6));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        mControllerManager.invokeHandlers();

        drive();
        pollLimbInputs();
    }

    void pollLimbInputs() {
        var operatorJoystick = mControllerManager.getOperatorJoystick();
        var rotation = -operatorJoystick.getYWithDeadZone();
        var extension = operatorJoystick.getXWithDeadZone();

        if (mArm.getControlState() != ArmControlState.TARGET) {
            mArm.setRotationRawDemand(rotation);
            mArm.setExtensionRawDemand(extension);
        }
    }

    private void drive() {
        var throttle = -mControllerManager.getDriverJoystick().getY();
        var strafe = -mControllerManager.getDriverJoystick().getX();
        var rot = -mControllerManager.getDriverJoystick().getZWithDeadZone();
        var mag = Math.sqrt(throttle * throttle + strafe * strafe);

        if (mSuperstructure.isSwerveLocking()) {
            return;
        }

        if (mSuperstructure.getAlliance() != Alliance.Blue) {
            throttle = -throttle;
            strafe = -strafe;
        }

        if (mag < Constants.kDriverXDeadZone) {
            mDrive.setVelocity(new ChassisSpeeds(0, 0, rot * Constants.kMaxAngularVelocityRadiansPerSecond));
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    throttle * Constants.kMaxVelocityMetersPerSecond,
                    strafe * Constants.kMaxVelocityMetersPerSecond,
                    rot * Constants.kMaxAngularVelocityRadiansPerSecond,
                    mDrive.getFieldRelativeGyroscopeRotation()));
        }
    }

    @Override
    public void disabledInit() {
        mVision.setLimelightMode(LimelightMode.PIECE_DETECTION);
        mLED.setDriveTeamColor(LEDState.kRobotStateDisabled);
        mLED.setHumanPlayerColor(LEDState.kRobotStateDisabled);
        mEnabledLooper.stop();
        mDisabledLooper.start();
    }

    @Override
    public void disabledPeriodic() {
        mDrive.zeroGyroscope(mPoseEstimator.getCurrentPose().getRotation());
        mArm.zeroArmRotation();
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
