package com.team1701.lib.util;

import com.team1701.lib.drivers.LEDController.LightColor;

public class LEDState {
    public static final LightColor kNone = LightColor.BLACK;
    public static final LightColor kRobotStateNone = LightColor.RED;
    public static final LightColor kRobotStateNoneCube = LightColor.VIOLET;
    public static final LightColor kRobotStateNoneCone = LightColor.YELLOW;
    public static final LightColor kRobotTrackingPieceInsideRange = LightColor.WHITE;
    public static final LightColor kRobotStateSubstation = LightColor.GREEN;
    public static final LightColor kRobotStateDisabled = LightColor.RED;
    public static final LightColor kRobotStateTrackedPieceOutsideRange = LightColor.ORANGE;
    public static final LightColor kRobotSeesPiece = LightColor.BLUE;
    public static final LightColor kRobotHasPiece = LightColor.GREEN;
}
