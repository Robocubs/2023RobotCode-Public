package com.team1701.frc2023.subsystems;

import java.util.Map;
import java.util.Set;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.frc2023.subsystems.Drive.DriveControlState;
import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake.IntakeState;
import com.team1701.frc2023.subsystems.OperatorInterfaceClient.INTFACTION;
import com.team1701.lib.arm.ArmSetpoint;
import com.team1701.lib.field.AprilTagSection;
import com.team1701.lib.field.ConeColumn;
import com.team1701.lib.intake.HandoffState;
import com.team1701.lib.intake.IntakeHandoffController;
import com.team1701.lib.util.LEDState;
import com.team1701.lib.util.LatchedBoolean;
import com.team1701.lib.util.TimeLockedBoolean;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends Subsystem {
    private final LoggedDashboardChooser<DriverStation.Alliance> mAllianceChooser =
            new LoggedDashboardChooser<>("Alliance");
    private final LoggedDashboardChooser<BasicFeatureFlag> mArmPremoveEnableChooser =
            new LoggedDashboardChooser<>("AK Logs Enabled");
    private final LoggedDashboardChooser<BasicFeatureFlag> mHandoffEnableChooser =
            new LoggedDashboardChooser<>("Arm Premoving");
    private final LoggedDashboardChooser<BasicFeatureFlag> mEnableAKLogsChooser =
            new LoggedDashboardChooser<>("Intake Handoff");
    private DriverStation.Alliance mAlliance = DriverStation.Alliance.Invalid;

    /* START INSTANCES */

    private static Superstructure mInstance = null;
    private final Drive mDrive = Drive.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Hand mHand = Hand.getInstance();
    private final LED mLED = LED.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
    private final Vision mVision = Vision.getInstance();

    /* END INSTANCES */

    /* START STATE FIELDS */

    private RobotState mRobotState = RobotState.NONE;
    private SwerveLockState mSwerveLockState = SwerveLockState.DISENGAGED;
    private TargetType mTargetType = TargetType.NONE;
    private IntakeHandoffController mIntakeHandoffController = new IntakeHandoffController();

    /* END STATE FIELDS */

    /* START OTHER */
    private TargetType mMainOperatorPositioningType = TargetType.NONE;
    private TargetType mPremovePositioningType = TargetType.NONE;
    private Pose2d mMainOperatorWaypoint = new Pose2d();
    private boolean mIsTargetTypeAuto = false;
    private RobotState mAutoTargetRobotState = RobotState.NONE;
    private LatchedBoolean mCanPremoveArm = new LatchedBoolean();
    private TimeLockedBoolean mSeesGamePiece = new TimeLockedBoolean(0.5, 0);
    private boolean mAllowArmPremoving = false;
    private boolean mAllowHandoff = true;

    public static enum RobotState {
        NONE,
        // Requesting or engaging swerve lock
        SWERVE_LOCK,
        // Picking pieces up from the Substation
        SUBSTATION_PICKUP,
        // Positioning to target AND scoring
        SCORING
    }

    public static enum SwerveLockState {
        // Swerve lock is not currently active
        DISENGAGED,
        // Swerve lock has been requested, but the robot is going too fast, so it is slowing down
        REQUESTED,
        // Swerve lock is fully engaged
        ENGAGED
    }

    public static enum TargetType {
        NONE,
        HIGH_CONE,
        HIGH_CONE_FLOOR,
        HIGH_CONE_REVERSE,
        HIGH_CUBE,
        HIGH_CUBE_REVERSE,
        MID_CONE,
        MID_CUBE,
        MID_CUBE_REVERSE,
        LOW,
        FLOOR_PICKUP,
        SUBSTATION,
        HOLD,
        PREMOVE_OPERATOR,
        HANDOFF_READY,
        HANDOFF_GRAB
    }

    public static enum BasicFeatureFlag {
        ALLOW,
        DISALLOW
    }

    public static enum ArmPremovePositionState {
        NONE,
        SUBSTATION,
        COMMUNITY
    }

    private final Set<TargetType> kScoringTargetTypes = Set.of(
            TargetType.HIGH_CONE,
            TargetType.HIGH_CONE_FLOOR,
            TargetType.HIGH_CUBE,
            TargetType.HIGH_CUBE_REVERSE,
            TargetType.HIGH_CONE_REVERSE,
            TargetType.MID_CONE,
            TargetType.MID_CUBE,
            TargetType.MID_CUBE_REVERSE,
            TargetType.LOW);

    private final Map<Integer, TargetType> kConeSet =
            Map.of(0, TargetType.LOW, 1, TargetType.MID_CONE, 2, TargetType.HIGH_CONE);
    private final Map<Integer, TargetType> kCubeSet =
            Map.of(0, TargetType.LOW, 1, TargetType.MID_CUBE, 2, TargetType.HIGH_CUBE);
    /* END OTHER */

    /* START GOAL POSITIONS */
    // Order is L, M, R
    private boolean mScoringPositionsInited = false;
    private AprilTagSection mSections[];
    /* END GOAL POSITIONS */

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    public Superstructure() {
        mEnableAKLogsChooser.addDefaultOption("ALLOW", BasicFeatureFlag.ALLOW);
        mEnableAKLogsChooser.addOption("DISALLOW", BasicFeatureFlag.DISALLOW);

        mArmPremoveEnableChooser.addDefaultOption("ALLOW", BasicFeatureFlag.ALLOW);
        mArmPremoveEnableChooser.addOption("DISALLOW", BasicFeatureFlag.DISALLOW);

        mHandoffEnableChooser.addDefaultOption("ALLOW", BasicFeatureFlag.ALLOW);
        mHandoffEnableChooser.addOption("DISALLOW", BasicFeatureFlag.DISALLOW);

        mAllianceChooser.addDefaultOption("FMS", null);
        mAllianceChooser.addOption("BLUE", DriverStation.Alliance.Blue);
        mAllianceChooser.addOption("RED", DriverStation.Alliance.Red);
    }

    public void setHandoffCompletionTarget(TargetType completionTarget) {
        mIntakeHandoffController.setCompletionTarget(completionTarget);
    }

    private void pollAndDoHandoff() {
        if (mHandoffEnableChooser.get() != BasicFeatureFlag.ALLOW
                || mMainOperatorPositioningType == TargetType.LOW
                || !mAllowHandoff) {
            if (mMainOperatorPositioningType == TargetType.LOW
                    && mIntake.hasPiece()
                    && mIntake.getIntakeState() == IntakeState.INTAKING) {
                mIntake.setIntakeState(IntakeState.STOW);
            }
            return;
        }
        HandoffState nextHandoffState =
                mIntakeHandoffController.doNextHandoffStage(getMeasuredHandoffstate(), Timer.getFPGATimestamp());
        if (nextHandoffState.armTargetType != TargetType.NONE) {
            setTargetType(nextHandoffState.armTargetType);
        }
        if (nextHandoffState.intakeState != IntakeState.NONE) {
            mIntake.setIntakeState(nextHandoffState.intakeState);
        }
        if (nextHandoffState.handState != HandState.NONE) {
            mHand.setHandState(nextHandoffState.handState);
        }
        return;
    }

    private void pollAndDoPreMoveArm() {
        if (!mAllowArmPremoving
                || mTargetType != TargetType.HOLD
                || mIntake.getIntakeState() == IntakeState.INTAKING
                || mIntake.hasPiece()) {
            return;
        }

        /*
         * If the robot is:
         * 1. Just now entering the community or substation
         *  >>> 2. In posession of a game-piece or not (Handled by getArmPremovePositioningState, and depends on the state)
         * 3. Operating during teleop
         * 4. NOT targetting LOW
         *
         * then premove arm to ONLY the target angle.
         */
        if (mCanPremoveArm.update(getArmPremovePositioningState() != ArmPremovePositionState.NONE)
                && DriverStation.isTeleopEnabled()) {
            setTargetType(TargetType.PREMOVE_OPERATOR);
        }
    }

    public ArmPremovePositionState getArmPremovePositioningState() {
        var currentX = mPoseEstimator.getCurrentPose().getX();
        var currentY = mPoseEstimator.getCurrentPose().getY();
        var state = ArmPremovePositionState.NONE;
        var handHasObject = mHand.seesObject() || mHand.isClosed();
        var canPremoveInCommunity = mMainOperatorPositioningType != TargetType.LOW;

        /*
         * Note for the line-hungry amongst you:
         * This could be split into a seperate method to squeeze line count, but the logic flow is easier to read this
         * way
         *
         * Note: We can set the mPremovePositioningType (where the Arm targets) ONLY when the substation is entered.
         * This is because that is a static angle no matter what. When entering the community, the premove code relies
         * on the operator to already have set the piece-to-be scored because that is a DYNAMIC angle (HIGH VS LOW VS MID).
         */
        if (mAlliance == Alliance.Red) {
            if (currentX > Constants.kRedCommunityDelimeterX
                    && handHasObject
                    && currentY < Constants.kRedCommunityDelimeterY
                    && mMainOperatorPositioningType != TargetType.LOW
                    && canPremoveInCommunity) {
                state = ArmPremovePositionState.COMMUNITY;
                mPremovePositioningType = mMainOperatorPositioningType;
            } else if (currentX < Constants.kBlueCommunityDelimeterX
                    && !handHasObject
                    && currentY > Constants.kRedCommunityDelimeterY) {
                mPremovePositioningType = TargetType.SUBSTATION;
                state = ArmPremovePositionState.SUBSTATION;
            } else {
                mPremovePositioningType = TargetType.NONE;
            }
        } else if (mAlliance == Alliance.Blue) {
            if (currentX < Constants.kBlueCommunityDelimeterX
                    && handHasObject
                    && currentY < Constants.kBlueCommunityDelimeterY
                    && canPremoveInCommunity) {
                state = ArmPremovePositionState.COMMUNITY;
                mPremovePositioningType = mMainOperatorPositioningType;
            } else if (currentX > Constants.kRedCommunityDelimeterX
                    && !handHasObject
                    && currentY > Constants.kBlueCommunityDelimeterY) {
                mPremovePositioningType = TargetType.SUBSTATION;
                state = ArmPremovePositionState.SUBSTATION;
            } else {
                mPremovePositioningType = TargetType.NONE;
            }
        }
        return state;
    }

    private void constructAprilTagSections() {
        int ids[];
        if (mAlliance != DriverStation.Alliance.Invalid && !mScoringPositionsInited) {
            if (mAlliance == DriverStation.Alliance.Red) {
                ids = Constants.kRedAprilTagIds;
            } else {
                ids = Constants.kBlueAprilTagIds;
            }
            mSections = new AprilTagSection[3];
            mSections[0] = new AprilTagSection(ids[0], 0);
            mSections[1] = new AprilTagSection(ids[1], 3);
            mSections[2] = new AprilTagSection(ids[2], 6);
            mScoringPositionsInited = true;
        }
    }

    public void setTargetType(TargetType targetType) {
        setTargetType(targetType, false);
    }

    public void setTargetType(TargetType targetType, boolean premove) {
        mTargetType = targetType;
        if (kScoringTargetTypes.contains(targetType) && !premove) {
            mRobotState = RobotState.SCORING;
        } else if (targetType == TargetType.SUBSTATION) {
            mRobotState = RobotState.SUBSTATION_PICKUP;
        } else {
            resetRobotState();
        }

        if (targetType == TargetType.PREMOVE_OPERATOR) {
            var operatorPositioningType = mPremovePositioningType;
            if (operatorPositioningType != null && operatorPositioningType == TargetType.SUBSTATION) {
                mArm.setTarget(Constants.kArmSetpoints.get(TargetType.SUBSTATION));
                mHand.setHandState(HandState.AUTO_CLOSE);
            } else if (operatorPositioningType != null && operatorPositioningType != TargetType.NONE) {
                mArm.setSetpoint(new ArmSetpoint(
                        Constants.kArmKinematics.toArmSetpoint(Constants.kArmSetpoints.get(operatorPositioningType))
                                .angle,
                        0));
            } else {
                DriverStation.reportWarning("Superstructure mPremovePositioningType is null", false);
            }
        } else if (Constants.kArmSetpoints.containsKey(targetType)) {
            var target = Constants.kArmSetpoints.get(targetType);
            if (premove) {
                var setpoint = Constants.kArmKinematics.toArmSetpoint(target);
                mArm.setSetpoint(new ArmSetpoint(setpoint.angle, 0));
            } else {
                mArm.setTarget(target);
            }
        } else {
            mArm.setArmControlStateNone();
        }
    }

    public void startAutoPosition() {
        mIsTargetTypeAuto = true;
        mRobotState = mAutoTargetRobotState;
    }

    public void stopAutoPosition() {
        mIsTargetTypeAuto = false;
        mRobotState = RobotState.NONE;
        mDrive.setDriveControlState(DriveControlState.VELOCITY_CONTROL);
        mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
    }

    public void requestOperatorAction(OperatorInterfaceClient.INTFACTION action) {
        if (!mScoringPositionsInited) {
            return;
        }

        var shouldInterruptHandoff = true;

        if (action == INTFACTION.SUBLEFT || action == INTFACTION.SUBRIGHT) {
            mMainOperatorPositioningType = TargetType.SUBSTATION;
            mAutoTargetRobotState = RobotState.SUBSTATION_PICKUP;
            shouldInterruptHandoff = false;

            var substationTagId = mAlliance == Alliance.Red ? 5 : 4;
            var aprilTagPose = Vision.getInstance()
                    .getLoadedLayout()
                    .getTagPose(substationTagId)
                    .get()
                    .toPose2d();

            var xTranslation = Constants.kRobotLengthWithBumpers / 2;
            var yTranslation = action == INTFACTION.SUBLEFT ? -0.6 : 0.6;
            var aprilTagToRobotTransform =
                    new Transform2d(new Translation2d(xTranslation, yTranslation), Rotation2d.fromDegrees(180));

            mMainOperatorWaypoint = aprilTagPose.transformBy(aprilTagToRobotTransform);
        } else if (action == INTFACTION.PIECE) {
            shouldInterruptHandoff = false;
            mAutoTargetRobotState = RobotState.SCORING;

            var client = OperatorInterfaceClient.getInstance();
            var columnNumber = client.mColumnNumber;
            var rowNumber = client.mRowNumber;

            for (int sectionsIdx = 0; sectionsIdx < 3; sectionsIdx++) {
                var sectionColumnMap = mSections[sectionsIdx].mColumns;
                if (sectionColumnMap.containsKey(columnNumber)) {
                    var column = sectionColumnMap.get(columnNumber);
                    Map<Integer, TargetType> set;
                    if (column instanceof ConeColumn) {
                        set = kConeSet;
                    } else {
                        set = kCubeSet;
                    }
                    mMainOperatorPositioningType = set.get(rowNumber);
                    mMainOperatorWaypoint = column.kLineupPoint;
                    break;
                }
            }
        } else if (action == INTFACTION.POSITION) {
            // POSITION now acts as a positioning button for both scoring the piece and moving the arm to the substation
            // pickup state
            if (getArmPremovePositioningState() == ArmPremovePositionState.SUBSTATION) {
                setTargetType(TargetType.SUBSTATION);
                mHand.setHandState(HandState.AUTO_CLOSE);
            } else {
                setTargetType(mMainOperatorPositioningType);
            }
        } else if (action == INTFACTION.ARMHOME) {
            setTargetType(TargetType.HOLD);
        } else if (action == INTFACTION.CLAWTOGGLE) {
            shouldInterruptHandoff = false;
            restartHandoffStateMachine();
            mHand.toggleHandState();
        } else if (action == INTFACTION.INTAKEEXTEND) {
            shouldInterruptHandoff = false;
            restartHandoffStateMachine();
            mIntake.setIntakeState(IntakeState.INTAKING);
        } else if (action == INTFACTION.INTAKEREJECT) {
            mIntake.setIntakeState(IntakeState.REJECTING);
        } else if (action == INTFACTION.INTAKERETRACT) {
            mIntake.setIntakeState(IntakeState.STOW);
        } else if (action == INTFACTION.INTAKETOGGLE) {
            mIntake.setIntakeState(IntakeState.STOP);
        } else if (action == INTFACTION.COMPRESS) {
            Pneumatics.getInstance().doInstantHackyCompression();
        } else if (action == INTFACTION.DUNK) {
            var currentArmTranslation =
                    Constants.kArmKinematics.toPose(mArm.getMeasuredSetpoint()).getTranslation();
            var dunkTranslation = new Translation2d(currentArmTranslation.getX(), currentArmTranslation.getY() - 0.10);
            mArm.setTarget(dunkTranslation);
        } else if (action == INTFACTION.INVALID) {
            mMainOperatorPositioningType = TargetType.NONE;
            mAutoTargetRobotState = RobotState.NONE;
            System.out.println("Superstructure operator action requested with INVALID state!");
        } else {
            mAutoTargetRobotState = RobotState.NONE;
        }
        if (shouldInterruptHandoff) {
            interruptHandoffStateMachine();
        }
    }

    public void resetRobotState() {
        if (mRobotState != RobotState.SWERVE_LOCK) {
            mRobotState = RobotState.NONE;
        }
    }

    public RobotState getRobotState() {
        return mRobotState;
    }

    public DriverStation.Alliance getAlliance() {
        return mAlliance;
    }

    public SwerveLockState getSwerveLockState() {
        return mSwerveLockState;
    }

    public boolean isFacingForward() {
        var headingDegrees =
                MathUtil.inputModulus(mDrive.getFieldRelativeGyroscopeRotation().getDegrees(), -180, 180);
        var isForward = headingDegrees > -90 && headingDegrees < 90;
        if (DriverStation.getAlliance() == Alliance.Red) {
            isForward = !isForward;
        }
        return isForward;
    }

    public HandoffState getMeasuredHandoffstate() {
        return new HandoffState(
                mTargetType,
                mIntake.getIntakeState(),
                mHand.getHandState(),
                mArm.atTarget(),
                mIntake.atTarget(),
                mIntake.hasPiece(),
                mHand.isClosed());
    }

    public void engageSwerveLock() {
        mRobotState = RobotState.SWERVE_LOCK;
        mSwerveLockState = SwerveLockState.REQUESTED;
    }

    public void disengageSwerveLock() {
        mSwerveLockState = SwerveLockState.DISENGAGED;
    }

    public boolean isSwerveLocking() {
        return mRobotState == RobotState.SWERVE_LOCK;
    }

    public void interruptHandoffStateMachine() {
        mIntakeHandoffController.interruptOperation();
    }

    public void restartHandoffStateMachine() {
        mIntakeHandoffController.clearInterrupt();
    }

    public void disableHandoff() {
        mAllowHandoff = false;
    }

    public void enableHandoff() {
        mAllowHandoff = true;
    }

    private void onLoopStateNone() {}

    private void onLoopStateSwerveLock() {
        switch (mSwerveLockState) {
            case DISENGAGED:
                // Exit mRobotState : SWERVE_LOCK
                mRobotState = RobotState.NONE;
                break;
            case ENGAGED:
            case REQUESTED:
                // Rotate modules to 'X' configuration
                mDrive.engageSwerveLock();
                break;
            default:
                break;
        }
    }

    private void handleLoopStateAuto() {
        if (mMainOperatorWaypoint == null) {
            stopAutoPosition();
            return;
        }
        mDrive.setWaypoint(mMainOperatorWaypoint);
    }

    public void onLoopStateScoring() {
        if (mIsTargetTypeAuto) {
            handleLoopStateAuto();
        }
    }

    private void onLoopStateSubstationPickup() {
        if (mIsTargetTypeAuto) {
            handleLoopStateAuto();
        }
    }

    /* END LOOP STATE HANDLERS */

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRobotState = RobotState.NONE;
                Brakes.getInstance().disengageExtensionBrake();
                if (mMainOperatorPositioningType == TargetType.NONE) {
                    mLED.setDriveTeamColor(LEDState.kRobotStateNone);
                    mLED.setHumanPlayerColor(LEDState.kRobotStateNone);
                }
                setHandoffCompletionTarget(TargetType.HOLD);
            }

            @Override
            public void onLoop(double timestamp) {
                mSeesGamePiece.update(mVision.limelightSeesTarget(), timestamp);
                if (kCubeSet.containsValue(mMainOperatorPositioningType)) {
                    mLED.setHumanPlayerColor(LEDState.kRobotStateNoneCube);
                } else if (kConeSet.containsValue(mMainOperatorPositioningType)) {
                    mLED.setHumanPlayerColor(LEDState.kRobotStateNoneCone);
                }
                constructAprilTagSections();
                pollAndDoHandoff();
                pollAndDoPreMoveArm();
                switch (mRobotState) {
                    case NONE:
                        onLoopStateNone();
                        break;
                    case SWERVE_LOCK:
                        onLoopStateSwerveLock();
                        break;
                    case SCORING:
                        onLoopStateScoring();
                        break;
                    case SUBSTATION_PICKUP:
                        onLoopStateSubstationPickup();
                        break;
                    default:
                        break;
                }

                if (mSeesGamePiece.getValue()) {
                    if (PieceSeeker.getInstance().getPieceFilterPass()) {
                        mLED.setDriveTeamColor(LEDState.kRobotTrackingPieceInsideRange);

                    } else {
                        mLED.setDriveTeamColor(LEDState.kRobotStateTrackedPieceOutsideRange);
                    }
                } else if (mHand.isClosed()) {
                    mLED.setDriveTeamColor(LEDState.kRobotHasPiece);
                } else if (mHand.seesObject()) {
                    mLED.setDriveTeamColor(LEDState.kRobotSeesPiece);
                } else {
                    mLED.setDriveTeamColor(LEDState.kRobotStateNone);
                }

                mLED.setReversed(isFacingForward());
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Superstructure.class.getSimpleName();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        if (mEnableAKLogsChooser.get() == BasicFeatureFlag.DISALLOW) {
            Logger.getInstance().end();
        }
        mAllowArmPremoving = mArmPremoveEnableChooser.get() == BasicFeatureFlag.ALLOW;
        var selectedAlliance = mAllianceChooser.get();
        var useFMSAlliance = DriverStation.isFMSAttached() || selectedAlliance == null;
        var newAlliance = useFMSAlliance ? DriverStation.getAlliance() : selectedAlliance;
        if (newAlliance != mAlliance) {
            // If the new alliance is different, we should recalculate the AprilTag sections
            mScoringPositionsInited = false;
        }
        mAlliance = newAlliance;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        var logger = Logger.getInstance();
        logger.recordOutput(
                "Superstructure/HandoffControllerState",
                mIntakeHandoffController.getHandoffControllerState().toString());
        logger.recordOutput("Superstructure/MainRobotState", mRobotState.toString());
        logger.recordOutput("Superstructure/AutoPositionMainRobotState", mAutoTargetRobotState.toString());
        logger.recordOutput("Superstructure/SwerveLockState", mSwerveLockState.toString());
        logger.recordOutput("Superstructure/TargetType", mTargetType.toString());

        // For now
        if (mMainOperatorWaypoint != null) {
            logger.recordOutput("Superstructure/OperatorWaypointPose", mMainOperatorWaypoint);
        }

        logger.recordOutput("Superstructure/OperatorType", mMainOperatorPositioningType.toString());
        logger.recordOutput("Superstructure/PremoveType", mPremovePositioningType.toString());
    }
}
