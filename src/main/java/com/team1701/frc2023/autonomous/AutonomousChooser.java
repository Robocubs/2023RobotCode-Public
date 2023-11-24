package com.team1701.frc2023.autonomous;

import com.team1701.frc2023.autonomous.AutonomousModes.BumpPathChoice;
import com.team1701.frc2023.autonomous.AutonomousModes.NoBumpPathChoice;
import com.team1701.frc2023.subsystems.Superstructure.TargetType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousChooser {

    private static AutonomousChooser mInstance;
    private final LoggedDashboardChooser<AutonomousBehavior> mAutoBehaviorChooser =
            new LoggedDashboardChooser<>("autoBEHAVIOR");
    private final LoggedDashboardChooser<AutonomousPickupMethod> mAutoPickupMethodChooser =
            new LoggedDashboardChooser<>("autoPICKUPMETHOD");

    public static synchronized AutonomousChooser getInstance() {
        if (mInstance == null) {
            mInstance = new AutonomousChooser();
        }

        return mInstance;
    }

    private AutonomousChooser() {
        // Intake method Chooser
        mAutoPickupMethodChooser.addDefaultOption("DEPLOYABLE_INTAKE", AutonomousPickupMethod.DEPLOYABLE_INTAKE);
        mAutoPickupMethodChooser.addOption("ARM", AutonomousPickupMethod.ARM);

        // Behavior Chooser
        mAutoBehaviorChooser.addDefaultOption("DO ABSOLUTELY NOTHING", AutonomousBehavior.DO_ABSOLUTELY_NOTHING);
        mAutoBehaviorChooser.addOption("JUST SCORE", AutonomousBehavior.JUST_SCORE);
        mAutoBehaviorChooser.addOption("CENTER DOCK", AutonomousBehavior.CENTER_DOCK);
        mAutoBehaviorChooser.addOption("BUMP SCORE TWO", AutonomousBehavior.BUMP_SCORE_TWO);
        mAutoBehaviorChooser.addOption("BUMP DOCK", AutonomousBehavior.BUMP_SCORE_TWO_DOCK);
        mAutoBehaviorChooser.addOption("NO BUMP SCORE TWO", AutonomousBehavior.NO_BUMP_SCORE_TWO);
        mAutoBehaviorChooser.addOption("NO BUMP DOCK", AutonomousBehavior.NO_BUMP_SCORE_TWO_DOCK);
        mAutoBehaviorChooser.addOption("NO BUMP SCORE THREE", AutonomousBehavior.NO_BUMP_SCORE_THREE);
        mAutoBehaviorChooser.addOption("BUMP SCORE THREE", AutonomousBehavior.BUMP_SCORE_THREE);
        mAutoBehaviorChooser.addOption("CENTER DOCK PICKUP", AutonomousBehavior.CENTER_DOCK_PICKUP);
        mAutoBehaviorChooser.addOption("CENTER DOCK SCORE TWO", AutonomousBehavior.CENTER_DOCK_SCORE_TWO);
    }

    public AutonomousMode getSelectedMode() {
        var behaviorSelected = mAutoBehaviorChooser.get();
        var pickupMethod = mAutoPickupMethodChooser.get();

        switch (behaviorSelected) {
            case JUST_SCORE:
                return AutonomousModes.justScore(TargetType.HIGH_CUBE);
            case CENTER_DOCK:
                return AutonomousModes.centerDock();
            case CENTER_DOCK_PICKUP:
                if (pickupMethod != AutonomousPickupMethod.DEPLOYABLE_INTAKE) {
                    return AutonomousModes.justScore(TargetType.HIGH_CUBE);
                } else {
                    return AutonomousModes.centerDockPickup();
                }
            case CENTER_DOCK_SCORE_TWO:
                if (pickupMethod != AutonomousPickupMethod.DEPLOYABLE_INTAKE) {
                    return AutonomousModes.justScore(TargetType.HIGH_CUBE);
                } else {
                    return AutonomousModes.centerDockScoreTwo();
                }
            case BUMP_SCORE_TWO:
                if (pickupMethod == AutonomousPickupMethod.ARM) {
                    return AutonomousModes.bump(BumpPathChoice.SCORE_TWO);
                } else {
                    return AutonomousModes.bumpIntake(BumpPathChoice.SCORE_TWO_INTAKE);
                }
            case BUMP_SCORE_TWO_DOCK:
                if (pickupMethod == AutonomousPickupMethod.ARM) {
                    return AutonomousModes.bump(BumpPathChoice.SCORE_TWO_DOCK);
                } else {
                    return AutonomousModes.bumpIntake(BumpPathChoice.SCORE_TWO_DOCK_INTAKE);
                }
            case NO_BUMP_SCORE_TWO:
                if (pickupMethod == AutonomousPickupMethod.ARM) {
                    return AutonomousModes.noBump(NoBumpPathChoice.SCORE_TWO);
                } else {
                    return AutonomousModes.noBumpIntake(NoBumpPathChoice.SCORE_TWO_INTAKE);
                }
            case NO_BUMP_SCORE_TWO_DOCK:
                if (pickupMethod == AutonomousPickupMethod.ARM) {
                    return AutonomousModes.noBump(NoBumpPathChoice.SCORE_TWO_DOCK);
                } else {
                    return AutonomousModes.noBumpIntake(NoBumpPathChoice.SCORE_TWO_DOCK_INTAKE);
                }
            case NO_BUMP_SCORE_THREE:
                return AutonomousModes.threePiece(false);
            case BUMP_SCORE_THREE:
                return AutonomousModes.threePiece(true);
            case DO_ABSOLUTELY_NOTHING:
            default:
                return AutonomousModes.none();
        }
    }

    public static enum AutonomousPickupMethod {
        ARM,
        DEPLOYABLE_INTAKE
    }

    public static enum AutonomousBehavior {
        DO_ABSOLUTELY_NOTHING,
        JUST_SCORE,
        CENTER_DOCK,
        BUMP_SCORE_TWO,
        BUMP_SCORE_TWO_DOCK,
        BUMP_SCORE_THREE,
        NO_BUMP_SCORE_TWO,
        NO_BUMP_SCORE_TWO_DOCK,
        NO_BUMP_SCORE_THREE,
        CENTER_DOCK_PICKUP,
        CENTER_DOCK_SCORE_TWO
    }
}
