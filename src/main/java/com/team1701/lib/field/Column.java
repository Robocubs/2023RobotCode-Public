package com.team1701.lib.field;

import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.frc2023.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Column {
    protected Vision mVision;
    public Pose2d kLineupPoint = new Pose2d();
    protected DriverStation.Alliance mAlliance;
    protected double kLineupRotation;

    public Column() {
        mVision = Vision.getInstance();
        mAlliance = Superstructure.getInstance().getAlliance();
        kLineupRotation = mAlliance == Alliance.Red ? 0.0 : 180.0;
    }
}
