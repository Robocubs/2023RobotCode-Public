package com.team1701.lib.intake;

import com.team1701.frc2023.subsystems.Hand.HandState;
import com.team1701.frc2023.subsystems.Intake.IntakeState;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;

public class HandoffState {
    // TODO: Clean this entire class up and organize states.
    public final TargetType armTargetType;
    public final IntakeState intakeState;
    public final HandState handState;

    public final boolean armAtTarget;
    public final boolean intakeAtTarget;
    public final boolean intakeHasPiece;
    public final boolean handIsClosed;

    public HandoffState() {
        this(TargetType.NONE, IntakeState.NONE, HandState.NONE);
    }

    public HandoffState(TargetType targetType, IntakeState intakeState, HandState handState) {
        this(targetType, intakeState, handState, false, false, false, false);
    }

    public HandoffState(
            TargetType targetType,
            IntakeState intakeState,
            HandState handState,
            boolean armAtTarget,
            boolean intakeAtTarget,
            boolean intakeHasPiece,
            boolean handIsClosed) {
        this.armTargetType = targetType;
        this.intakeState = intakeState;
        this.handState = handState;

        this.armAtTarget = armAtTarget;
        this.intakeAtTarget = intakeAtTarget;
        this.intakeHasPiece = intakeHasPiece;
        this.handIsClosed = handIsClosed;
    }

    @Override
    public String toString() {
        return "HandoffState(TargetType: " + armTargetType.toString() + " IntakeState: " + intakeState.toString()
                + " HandState: " + handState.toString() + " )";
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof HandoffState)) {
            return false;
        }

        var other = (HandoffState) obj;
        return armTargetType.equals(other.armTargetType)
                && intakeState.equals(other.intakeState)
                && handState.equals(other.handState);
    }
}
