package com.team1701.lib.field;

import com.team1701.frc2023.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ConeColumn extends Column {
    // Using vision because it already loads a field layout
    public ConeColumn(int tagID, boolean isLeft) {
        super();
        Pose3d columnTagPose3d =
                super.mVision.getLoadedLayout().getTagPose(tagID).get();
        double yNegation;
        if (isLeft) {
            yNegation = -1.0;
        } else {
            yNegation = 1.0;
        }

        var x = (((Constants.kRobotLengthWithBumpers / 2)) + Constants.kLineToTag);
        var y = 0.5615 * yNegation;

        var pose2dnorotation = columnTagPose3d
                .transformBy(new Transform3d(new Translation3d(x, y, -0.46119), new Rotation3d()))
                .toPose2d();
        super.kLineupPoint = new Pose2d(
                pose2dnorotation.getX(), pose2dnorotation.getY(), Rotation2d.fromDegrees(super.kLineupRotation));
    }
}
