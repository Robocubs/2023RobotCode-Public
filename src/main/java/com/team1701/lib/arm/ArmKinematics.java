package com.team1701.lib.arm;

import java.util.Optional;

import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmKinematics {
    private final Pose2d mPosition;
    private final Translation2d mHandPosition;

    public ArmKinematics(Pose2d position, Translation2d handPosition) {
        mPosition = position;
        mHandPosition = handPosition;
    }

    public ArmSetpoint toArmSetpoint(Translation2d point) {
        var translationFromPivot = point.minus(mPosition.getTranslation());
        var handExtendedPosition = calculateHandExtendedPosition(translationFromPivot.getNorm());
        var angle = translationFromPivot
                .getAngle()
                .minus(handExtendedPosition.getAngle())
                .minus(mPosition.getRotation());
        angle = Rotation2d.fromRadians(MathUtil.angleModulus(angle.getRadians()));
        var length = Math.max(handExtendedPosition.getX() - mHandPosition.getX(), 0);
        return new ArmSetpoint(angle, length);
    }

    public Pose2d toPose(ArmSetpoint setpoint) {
        return toPose(setpoint.angle, setpoint.extension);
    }

    public Pose2d toPose(Rotation2d angle, double extension) {
        var handExtendedPosition = mHandPosition.plus(new Translation2d(extension, 0));
        var angleRelativeToArmPosition =
                angle.plus(handExtendedPosition.getAngle()).plus(mPosition.getRotation());
        var translationFromPivot = new Translation2d(handExtendedPosition.getNorm(), angleRelativeToArmPosition);
        var translation = translationFromPivot.plus(mPosition.getTranslation());
        return new Pose2d(translation, angle);
    }

    public Optional<Rotation2d> closestAngleToHeightGivenSetpoint(ArmSetpoint setpoint, double height) {
        var handExtendedPosition = mHandPosition.plus(new Translation2d(setpoint.extension, 0));
        var length = handExtendedPosition.getNorm();

        if (Math.abs(height - mPosition.getY()) > length) {
            return Optional.empty();
        }

        var angle = Rotation2d.fromRadians(Math.asin((height - mPosition.getY()) / length))
                .plus(mHandPosition.getAngle())
                .minus(handExtendedPosition.getAngle())
                .minus(mPosition.getRotation());
        var alternateAngle = GeometryUtil.kRotationPi.minus(angle);

        var angleDiff = Math.abs(angle.minus(setpoint.angle).getRadians());
        var alternateAngleDiff = Math.abs(alternateAngle.minus(setpoint.angle).getRadians());

        return Optional.of(angleDiff < alternateAngleDiff ? angle : alternateAngle);
    }

    private Translation2d calculateHandExtendedPosition(double norm) {
        var handExtendedPositionX = Math.sqrt(norm * norm - mHandPosition.getY() * mHandPosition.getY());
        return new Translation2d(handExtendedPositionX, mHandPosition.getY());
    }
}
