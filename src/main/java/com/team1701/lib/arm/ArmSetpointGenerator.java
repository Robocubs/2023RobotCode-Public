package com.team1701.lib.arm;

import com.team1701.lib.util.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmSetpointGenerator {
    private final Rotation2d mAngleToFront;
    private final Rotation2d mAngleToBack;
    private final double mAngleToFrontRadians;
    private final double mAngleToBackRadians;
    private final ArmKinematics mKinematics;
    private final PassDirection mPassDirection;
    private final double mMaxExtension;
    private final double mMaxExtensionPassing;
    private final double mMeasuredRotationErrorMarginRadians;
    private final double mMeasuredExtensionErrorMargin;

    public ArmSetpointGenerator(
            Rotation2d angleToFront,
            Rotation2d angleToBack,
            ArmKinematics kinematics,
            PassDirection passDirection,
            double maxExtension,
            double maxExtensionPassing,
            Rotation2d measuredRotationErrorMarginRadians,
            double measuredExtensionErrorMargin) {
        mAngleToFront = angleToFront;
        mAngleToBack = angleToBack;
        mAngleToFrontRadians = MathUtil.angleModulus(angleToFront.getRadians());
        mAngleToBackRadians = MathUtil.angleModulus(angleToBack.getRadians());
        mKinematics = kinematics;
        mPassDirection = passDirection;
        mMaxExtension = maxExtension;
        mMaxExtensionPassing = maxExtensionPassing;
        mMeasuredRotationErrorMarginRadians = measuredRotationErrorMarginRadians.getRadians();
        mMeasuredExtensionErrorMargin = measuredExtensionErrorMargin;
    }

    public ArmSetpoint generateSetpoint(ArmSetpoint measuredSetpoint, Translation2d target) {
        return generateSetpoint(measuredSetpoint, mKinematics.toArmSetpoint(target));
    }

    /**
     * @param measuredSetpoint The measured state of the arm, such as from encoders.
     * @param desiredSetpoint The desired state of the arm.
     * @return An ArmSetpoint object that prevents collisions while converging to desiredSetpoint.
     */
    public ArmSetpoint generateSetpoint(ArmSetpoint measuredSetpoint, ArmSetpoint desiredSetpoint) {
        var measuredAngleRadians = MathUtil.angleModulus(measuredSetpoint.angle.getRadians());
        var desiredAngleRadians = MathUtil.angleModulus(desiredSetpoint.angle.getRadians());
        var passState = determinePassState(measuredAngleRadians, desiredAngleRadians);

        var setpoint = desiredSetpoint;
        setpoint = mPassDirection == PassDirection.THROUGH
                ? correctForPassThrough(measuredSetpoint, setpoint, passState)
                : correctForPassOver(measuredSetpoint, desiredSetpoint, passState);
        setpoint = applyRotationExtensionOrder(measuredSetpoint, setpoint, measuredAngleRadians, desiredAngleRadians);
        setpoint = applyMaxExtension(setpoint);

        return setpoint;
    }

    private ArmSetpoint correctForPassThrough(
            ArmSetpoint measuredSetpoint, ArmSetpoint desiredSetpoint, PassState passState) {
        Rotation2d angle;
        double extension;
        switch (passState) {
            case WILL_PASS_FROM_FRONT:
                angle = measuredSetpoint.extension > mMeasuredExtensionErrorMargin
                        ? mAngleToFront
                        : desiredSetpoint.angle;
                extension = mMaxExtensionPassing;
                break;
            case WILL_PASS_FROM_BACK:
                angle = measuredSetpoint.extension > mMeasuredExtensionErrorMargin
                        ? mAngleToBack
                        : desiredSetpoint.angle;
                extension = mMaxExtensionPassing;
                break;
            case PASSING:
                angle = measuredSetpoint.extension > mMeasuredExtensionErrorMargin
                        ? measuredSetpoint.angle
                        : desiredSetpoint.angle;
                extension = mMaxExtensionPassing;
                break;
            case NONE:
                return desiredSetpoint;
            default:
                return measuredSetpoint;
        }

        return new ArmSetpoint(angle, extension);
    }

    private ArmSetpoint correctForPassOver(
            ArmSetpoint measuredSetpoint, ArmSetpoint desiredSetpoint, PassState passState) {
        if (passState == PassState.NONE) {
            return desiredSetpoint;
        }

        var extension = Math.min(desiredSetpoint.extension, mMaxExtensionPassing);
        return new ArmSetpoint(desiredSetpoint.angle, extension);
    }

    private ArmSetpoint applyRotationExtensionOrder(
            ArmSetpoint measuredSetpoint,
            ArmSetpoint desiredSetpoint,
            double measuredAngleRadians,
            double desiredAngleRadians) {
        var rotationInPosition =
                Util.epsilonEquals(measuredAngleRadians, desiredAngleRadians, mMeasuredRotationErrorMarginRadians);
        var extensionInPosition = Util.epsilonEquals(
                measuredSetpoint.extension, desiredSetpoint.extension, mMeasuredExtensionErrorMargin);

        if (rotationInPosition || extensionInPosition) {
            return desiredSetpoint;
        }

        if (desiredSetpoint.extension < measuredSetpoint.extension) {
            return new ArmSetpoint(measuredSetpoint.angle, desiredSetpoint.extension);
        }

        return new ArmSetpoint(desiredSetpoint.angle, measuredSetpoint.extension);
    }

    public ArmSetpoint applyMaxExtension(ArmSetpoint desiredSetpoint) {
        if (Util.inRangeInclusive(desiredSetpoint.angle.getRadians(), mAngleToBackRadians, mAngleToFrontRadians)) {
            return new ArmSetpoint(desiredSetpoint.angle, 0);
        }

        if (desiredSetpoint.extension > mMaxExtension) {
            return new ArmSetpoint(desiredSetpoint.angle, mMaxExtension);
        }

        return desiredSetpoint;
    }

    private PassState determinePassState(double measuredAngleRadians, double desiredAngleRadians) {
        if (mPassDirection == PassDirection.OVER) {
            return Util.inRangeInclusive(Math.PI, measuredAngleRadians, desiredAngleRadians)
                    ? PassState.PASSING
                    : PassState.NONE;
        }

        if (Util.inRangeInclusive(measuredAngleRadians, mAngleToBackRadians, mAngleToFrontRadians)) {
            return PassState.PASSING;
        }

        if (Util.inRange(mAngleToBackRadians, measuredAngleRadians, desiredAngleRadians)) {
            return PassState.WILL_PASS_FROM_BACK;
        }

        if (Util.inRange(mAngleToFrontRadians, desiredAngleRadians, measuredAngleRadians)) {
            return PassState.WILL_PASS_FROM_FRONT;
        }

        return PassState.NONE;
    }

    public enum PassDirection {
        OVER,
        THROUGH,
    }

    private enum PassState {
        PASSING,
        WILL_PASS_FROM_FRONT,
        WILL_PASS_FROM_BACK,
        NONE
    }
}
