package com.team1701.lib.intake;

import com.team1701.frc2023.Constants;
import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake.IntakeState;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;
import com.team1701.lib.util.TimeLockedBoolean;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class IntakeHandoffController {
    private HandoffControllerState mHandoffControllerState = HandoffControllerState.NOT_HANDING_OFF;
    private TimeLockedBoolean mCanGoToNextHandoffState =
            new TimeLockedBoolean(Constants.kHandReachedSetpointBounceDuration, 0.0);
    private TimeLockedBoolean mHandoffSeesNoPiece = new TimeLockedBoolean(2.0, 0.0);
    private boolean mCanContinueFromPreviousIRQ = true;
    private boolean mIRQRequested = false;
    private TargetType mSequenceCompletionArmPos = TargetType.HOLD;

    public IntakeHandoffController() {}

    public void setCompletionTarget(TargetType completionTarget) {
        mSequenceCompletionArmPos = completionTarget;
    }

    public synchronized void interruptOperation() {
        if (mHandoffControllerState == HandoffControllerState.NOT_HANDING_OFF
                || mHandoffControllerState == HandoffControllerState.INTERRUPTED) {
            return;
        }
        DriverStation.reportWarning("Interrupting the handoff controller!", false);
        mIRQRequested = true;
        mHandoffControllerState = HandoffControllerState.INTERRUPTED;
        mCanContinueFromPreviousIRQ = false;
        return;
    }

    public synchronized void clearInterrupt() {
        if (mHandoffControllerState != HandoffControllerState.INTERRUPTED) {
            return;
        }
        DriverStation.reportWarning("Clearing the handoff controller interrupt!", false);
        mIRQRequested = false;
        resetHandoffstate();
        return;
    }

    public synchronized HandoffControllerState getHandoffControllerState() {
        return mHandoffControllerState;
    }

    public synchronized HandoffState doNextHandoffStage(HandoffState measuredState, double time) {
        if (mIRQRequested) {
            return new HandoffState();
        }
        switch (mHandoffControllerState) {
            case NOT_HANDING_OFF:
                if (!measuredState.intakeHasPiece && mHandoffSeesNoPiece.getValue()) {
                    mCanContinueFromPreviousIRQ = true;
                }
                if (!measuredState.intakeHasPiece || measuredState.handIsClosed) {
                    mHandoffSeesNoPiece.update(!measuredState.intakeHasPiece, time);
                    return new HandoffState();
                } else if (mCanContinueFromPreviousIRQ) {
                    // mHand.setHandState(HandState.OPEN);
                    // mIntake.setIntakeState(IntakeState.HANDOFF_HOLD);
                    mHandoffControllerState = HandoffControllerState.STARTING_HANDOFF;
                    return new HandoffState(TargetType.HANDOFF_READY, IntakeState.HANDOFF_HOLD, HandState.OPEN);
                } else {
                    mHandoffSeesNoPiece.update(!measuredState.intakeHasPiece, time);
                    return new HandoffState();
                }
            case STARTING_HANDOFF:
                if (measuredState.intakeAtTarget && measuredState.intakeState == IntakeState.HANDOFF_HOLD) {
                    mHandoffControllerState = HandoffControllerState.HANDOFF_READY_TO_GRAB;
                    mCanGoToNextHandoffState.update(false, time);
                    return new HandoffState();
                }
                return new HandoffState(TargetType.HANDOFF_READY, IntakeState.NONE, HandState.NONE);
            case HANDOFF_READY_TO_GRAB:
                mCanGoToNextHandoffState.update(
                        measuredState.intakeAtTarget && measuredState.intakeState == IntakeState.HANDOFF_HOLD, time);
                if (mCanGoToNextHandoffState.getValue()) {
                    mCanGoToNextHandoffState.update(false, time);
                    mHandoffControllerState = HandoffControllerState.HANDOFF_GRABBING;
                    return new HandoffState(TargetType.HANDOFF_GRAB, IntakeState.NONE, HandState.NONE);
                }
                return new HandoffState();
            case HANDOFF_GRABBING:
                mCanGoToNextHandoffState.update(measuredState.intakeAtTarget && measuredState.armAtTarget, time);
                if (measuredState.armAtTarget
                        && measuredState.armTargetType == TargetType.HANDOFF_GRAB
                        && mCanGoToNextHandoffState.getValue()) {
                    // mIntake.setIntakeState(IntakeState.HANDOFF_RELEASE);
                    // mHand.setHandState(HandState.CLOSE);
                    mHandoffControllerState = HandoffControllerState.HOMING_ARM;
                    return new HandoffState(TargetType.NONE, IntakeState.HANDOFF_RELEASE, HandState.CLOSE);
                }
                return new HandoffState();
            case HOMING_ARM:
                if (measuredState.handIsClosed) {
                    resetHandoffstate();
                    return new HandoffState(mSequenceCompletionArmPos, IntakeState.STOW, HandState.NONE);
                }
                return new HandoffState();
            default:
                return new HandoffState();
        }
    }

    private synchronized void resetHandoffstate() {
        mHandoffControllerState = HandoffControllerState.NOT_HANDING_OFF;
        mHandoffSeesNoPiece.update(false, Timer.getFPGATimestamp());
        return;
    }

    public enum HandoffControllerState {
        NOT_HANDING_OFF,
        STARTING_HANDOFF,
        HANDOFF_READY_TO_GRAB,
        HANDOFF_GRABBING,
        HOMING_ARM,
        INTERRUPTED
    }
}
