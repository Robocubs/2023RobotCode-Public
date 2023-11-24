package com.team1701.lib.field;

import com.team1701.frc2023.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CubeColumn extends Column {

    // Using vision because it already loads a field layout
    public CubeColumn(int tagID) {
        super();
        Pose3d columnTagPose3d =
                super.mVision.getLoadedLayout().getTagPose(tagID).get();

        var x = ((Constants.kRobotLengthWithBumpers / 2) + Constants.kLineToTag);

        var pose2dnorotation = columnTagPose3d
                .transformBy(new Transform3d(new Translation3d(x, 0.0, .46119), new Rotation3d()))
                .toPose2d();
        super.kLineupPoint = new Pose2d(
                pose2dnorotation.getX(), pose2dnorotation.getY(), Rotation2d.fromDegrees(super.kLineupRotation));
    }
}
