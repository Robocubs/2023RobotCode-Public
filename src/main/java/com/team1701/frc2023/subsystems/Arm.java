package com.team1701.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.lib.arm.ArmSetpoint;
import com.team1701.lib.arm.ArmSetpointGenerator;
import com.team1701.lib.arm.ArmSetpointGenerator.PassDirection;
import com.team1701.lib.drivers.RetroreflectiveSensor;
import com.team1701.lib.drivers.TalonFXFactory;
import com.team1701.lib.util.CTREEncoderConverter;
import com.team1701.lib.util.LatchedBoolean;
import com.team1701.lib.util.LoggingUtil;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Arm extends Subsystem {
    private static Arm mInstance = null;
    private final ArmInputsAutoLogged mPeriodicInputs = new ArmInputsAutoLogged();
    private final Brakes mBrakes = Brakes.getInstance();

    private ArmRotationState mArmRotationState = ArmRotationState.STOP;
    private ArmExtensionState mArmExtensionState = ArmExtensionState.STOP;
    private ArmControlState mArmControlState = ArmControlState.NONE;

    private TimeLockedBoolean mShouldEngageRotationBrake = new TimeLockedBoolean(0.5, Timer.getFPGATimestamp());
    // private TimeLockedBoolean mShouldEngageExtensionBrake = new TimeLockedBoolean(0.5, Timer.getFPGATimestamp());
    private LatchedBoolean mShouldZeroExtension = new LatchedBoolean();

    // Measured values
    private TimeLockedBoolean mAtRetractionLimit = new TimeLockedBoolean(0.05, Timer.getFPGATimestamp(), false);

    // Targeting
    private final ArmSetpointGenerator mArmSetpointGenerator;
    private ArmSetpoint mDesiredSetpoint = new ArmSetpoint();
    private Translation2d mTargetPoint = new Translation2d();
    private Mechanism2d mMechanism2d = new Mechanism2d(3, 3);
    private MechanismLigament2d mTelescope;

    // Motor demand
    private double mRotationDemand = 0;
    private double mExtensionDemand = 0;
    private ControlMode mRotationControlMode = ControlMode.PercentOutput;
    private ControlMode mExtensionControlMode = ControlMode.PercentOutput;

    private final TalonFX mRotationFalcon;
    private final TalonFX mExtensionFalcon;
    private final DutyCycleEncoder mThroughBoreEncoder;
    private final RetroreflectiveSensor mRetractionSensor;
    private final CTREEncoderConverter mCtreEncoderConverter =
            new CTREEncoderConverter(2048, Constants.kExtensionTalonReduction, Constants.kSpoolDiameterInches);

    @AutoLog
    public static class ArmInputs {
        public boolean atRetractionLimit;
        public double measuredAngleDegrees;
        public double measuredExtensionMeters;
        public double throughBoreEncoderDegrees;
    }

    /* START STATES */
    public enum ArmControlState {
        // Arm is not moving
        NONE,
        // Operator is using joysticks to manually control
        MANUAL,
        // PIDController's are being used to move arm
        TARGET
    }

    // This technically represents both rotation and height because height is a function of rotation in this case
    // Represents both clockwise and counter-clockwise rotation
    public enum ArmRotationState {
        STOP,
        ROTATING,
        ROTATED
    }

    // Represents both retraction and extension
    public enum ArmExtensionState {
        STOP,
        EXTENDING,
        EXTENDED
    }
    /* END STATES */

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    private Arm() {
        mRotationFalcon = TalonFXFactory.createTalon(Constants.kArmRotationTalonID, config -> {
            config.NEUTRAL_MODE = NeutralMode.Coast;
            config.INVERTED = true;

            config.forwardSoftLimitThreshold = toMotorPosition(Constants.kMaxArmRotation);
            config.forwardSoftLimitEnable = true;
            config.reverseSoftLimitThreshold = toMotorPosition(Constants.kMinArmRotation);
            config.reverseSoftLimitEnable = true;

            config.nominalOutputForward = 0.2;
            config.nominalOutputForward = -0.2;

            config.motionCruiseVelocity = Constants.kArmRotationCruiseVelocity;
            config.motionAcceleration = Constants.kArmRotationAcceleration;

            config.slot0 = new SlotConfiguration();
            config.slot0.kP = 0.1;
            config.slot0.kI = 0.0;
            config.slot0.kD = 2.0;
            config.slot0.allowableClosedloopError = toMotorPosition(0.5);
        });

        mExtensionFalcon = TalonFXFactory.createTalon(Constants.kArmExtensionTalonID, config -> {
            config.NEUTRAL_MODE = NeutralMode.Brake;

            config.forwardSoftLimitThreshold =
                    mCtreEncoderConverter.distanceToNativeUnits(Constants.kArmExtensionLimit);
            config.forwardSoftLimitEnable = true;
            config.reverseSoftLimitEnable = false;

            config.slot0 = new SlotConfiguration();
            config.slot0.kP = 0.1;
            config.slot0.kI = 0.0;
            config.slot0.kD = 0.0;
        });

        mThroughBoreEncoder = new DutyCycleEncoder(Constants.kArmThroughBoreEncoderID);
        mThroughBoreEncoder.setDistancePerRotation(360);

        mRetractionSensor = new RetroreflectiveSensor(Constants.kArmRetractionSensorID);

        var maxExtensionPassing = Constants.kArmPassDirection == PassDirection.OVER ? Constants.kMaxArmYPosition : 0;
        mArmSetpointGenerator = new ArmSetpointGenerator(
                Constants.kAngleToFront,
                Constants.kAngleToBack,
                Constants.kArmKinematics,
                Constants.kArmPassDirection,
                Constants.kArmExtensionLimit,
                maxExtensionPassing,
                Rotation2d.fromDegrees(45),
                0.1);
        mTelescope = mMechanism2d.getRoot("arm", 1.5, 1.5).append(new MechanismLigament2d("telescope", 0, 90));
    }

    public void zeroArmRotation() {
        // Sometimes the encoder starts up in a way that makes it appear to have a full rotation
        // We just assume that we are in the correct direction
        mRotationFalcon.setSelectedSensorPosition(toMotorPosition(mPeriodicInputs.throughBoreEncoderDegrees));
    }

    private double toArmDegrees(double motorPosition) {
        return normalizeDegrees(motorPosition / 2048.0 / Constants.kRotationTalonReduction * 360.0);
    }

    private double toMotorPosition(double armDegrees) {
        return normalizeDegrees(armDegrees) * 2048.0 * Constants.kRotationTalonReduction / 360.0;
    }

    private double toMotorPosition(Rotation2d armRotation) {
        return toMotorPosition(armRotation.getDegrees());
    }

    private double normalizeDegrees(double degrees) {
        return Constants.kArmPassDirection == PassDirection.THROUGH
                ? MathUtil.inputModulus(degrees, -180, 180)
                : MathUtil.inputModulus(degrees, 0, 360);
    }

    public ArmSetpoint getMeasuredSetpoint() {
        return new ArmSetpoint(
                Rotation2d.fromDegrees(mPeriodicInputs.measuredAngleDegrees), mPeriodicInputs.measuredExtensionMeters);
    }

    private ArmSetpoint getDemandSetpoint() {
        return new ArmSetpoint(Rotation2d.fromDegrees(toArmDegrees(mRotationDemand)), mExtensionDemand);
    }

    public void setExtensionRawDemand(double demand) {
        mArmControlState = ArmControlState.MANUAL;
        mArmExtensionState = ArmExtensionState.EXTENDING;
        mExtensionDemand = demand;
    }

    public void setRotationRawDemand(double demand) {
        mArmControlState = ArmControlState.MANUAL;
        mArmRotationState = ArmRotationState.ROTATING;
        if (Util.epsilonEquals(demand, 0.0)) {
            stopRotation();
        } else {
            mRotationDemand = demand;
        }
    }

    public void setTarget(Translation2d target) {
        var setpoint = Constants.kArmKinematics.toArmSetpoint(target);
        setSetpoint(setpoint);
        mTargetPoint = target;
    }

    public void setSetpoint(ArmSetpoint setpoint) {
        mTargetPoint = Constants.kArmKinematics.toPose(setpoint).getTranslation();
        mArmControlState = ArmControlState.TARGET;
        mDesiredSetpoint = mArmSetpointGenerator.applyMaxExtension(setpoint);
        mArmExtensionState = ArmExtensionState.EXTENDING;
        mArmRotationState = ArmRotationState.ROTATING;
    }

    public boolean atTarget() {
        return getExtensionState() == ArmExtensionState.EXTENDED && getRotationState() == ArmRotationState.ROTATED;
    }

    public ArmRotationState getRotationState() {
        return mArmRotationState;
    }

    public ArmExtensionState getExtensionState() {
        return mArmExtensionState;
    }

    public ArmControlState getControlState() {
        return mArmControlState;
    }

    public void stopRotation() {
        mArmRotationState = ArmRotationState.STOP;
        mRotationDemand = 0;
    }

    public void stopExtension() {
        mArmExtensionState = ArmExtensionState.STOP;
        mExtensionDemand = 0;
    }

    public void setArmControlStateNone() {
        mArmControlState = ArmControlState.NONE;
    }

    private void updateTargeting() {
        var armRotationState = ArmRotationState.ROTATING;
        var armExtensionState = ArmExtensionState.EXTENDING;
        mRotationControlMode = Constants.kArmRotationClosedLoopControlMode;
        mExtensionControlMode = ControlMode.Position;
        var measuredSetpoint = getMeasuredSetpoint();

        var generatedSetpoint = mArmSetpointGenerator.generateSetpoint(measuredSetpoint, mDesiredSetpoint);
        var onLastSetpoint = generatedSetpoint.equals(mDesiredSetpoint);
        var needToZero = Util.epsilonEquals(generatedSetpoint.extension, 0)
                && measuredSetpoint.extension <= Constants.kArmMeasuredExtensionErrorMargin
                && !mAtRetractionLimit.getValue();

        if (onLastSetpoint) {
            var angleError = MathUtil.angleModulus(
                    measuredSetpoint.angle.minus(mDesiredSetpoint.angle).getRadians());
            var atDesiredRotation = Util.inRange(angleError, Constants.kArmMeasuredRotationErrorMargin.getRadians());
            var atDesiredExtension = Util.epsilonEquals(
                    measuredSetpoint.extension, mDesiredSetpoint.extension, Constants.kArmMeasuredExtensionErrorMargin);
            if (atDesiredRotation) {
                armRotationState = ArmRotationState.ROTATED;
            }
            if (atDesiredExtension && !needToZero) {
                armExtensionState = ArmExtensionState.EXTENDED;
            }
        }

        if (needToZero) {
            mExtensionControlMode = ControlMode.PercentOutput;
            mExtensionDemand = -0.2;
            mRotationDemand = toMotorPosition(measuredSetpoint.angle);
        } else {
            mExtensionDemand = mCtreEncoderConverter.distanceToNativeUnits(generatedSetpoint.extension);
            mRotationDemand = toMotorPosition(generatedSetpoint.angle);
        }

        mArmRotationState = armRotationState;
        mArmExtensionState = armExtensionState;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                zeroArmRotation();
                mRotationFalcon.setNeutralMode(NeutralMode.Brake);
                mExtensionFalcon.setNeutralMode(NeutralMode.Brake);
                mExtensionControlMode = ControlMode.PercentOutput;
                mExtensionDemand = 0;
                mRotationControlMode = ControlMode.PercentOutput;
                mRotationDemand = 0;
            }

            @Override
            public void onLoop(double timestamp) {
                if (mArmRotationState == ArmRotationState.STOP && mArmExtensionState == ArmExtensionState.STOP) {
                    mArmControlState = ArmControlState.NONE;
                }
                /* START CONTROL HANDLING */
                switch (mArmControlState) {
                    case TARGET:
                        updateTargeting();
                        break;
                    case MANUAL:
                        mExtensionControlMode = ControlMode.PercentOutput;
                        mRotationControlMode = ControlMode.PercentOutput;
                        break;
                    case NONE:
                        mArmRotationState = ArmRotationState.STOP;
                        mArmExtensionState = ArmExtensionState.STOP;
                        break;
                }
                /* END CONTROL HANDLING */
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

            @Override
            public String getDisplayName() {
                return Arm.class.getSimpleName();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicInputs.atRetractionLimit = mRetractionSensor.get();
        mAtRetractionLimit.update(mPeriodicInputs.atRetractionLimit, Timer.getFPGATimestamp());
        mPeriodicInputs.throughBoreEncoderDegrees =
                mThroughBoreEncoder.getDistance() - Constants.kArmThroughBoreEncoderOffset;
        mPeriodicInputs.measuredAngleDegrees = toArmDegrees(mRotationFalcon.getSelectedSensorPosition());
        mPeriodicInputs.measuredExtensionMeters =
                mCtreEncoderConverter.nativeUnitsToDistanceMeters(mExtensionFalcon.getSelectedSensorPosition());
        Logger.getInstance().processInputs("Arm", mPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        var measured = getMeasuredSetpoint();
        mTelescope.setLength(measured.extension + 0.69);
        mTelescope.setAngle(measured.angle.minus(Rotation2d.fromDegrees(90)));
        var timestamp = Timer.getFPGATimestamp();
        switch (mArmRotationState) {
            case STOP:
                mRotationFalcon.set(ControlMode.PercentOutput, 0);
                mBrakes.engageRotationBrake();
                var measuredAngleNormalized = MathUtil.inputModulus(mPeriodicInputs.measuredAngleDegrees, 0, 360);
                if (mBrakes.rotationBrakeFinishedEngaging()
                        && measuredAngleNormalized > 170
                        && measuredAngleNormalized < 190) {
                    zeroArmRotation();
                }
                break;
            case ROTATED:
            case ROTATING:
                var shouldEngageBrake =
                        mShouldEngageRotationBrake.update(mArmRotationState == ArmRotationState.ROTATED, timestamp);
                if (shouldEngageBrake) {
                    mBrakes.engageRotationBrake();
                } else {
                    mBrakes.disengageRotationBrake();
                }

                if (mBrakes.rotationBrakeFinishedEngaging()) {
                    mRotationFalcon.set(ControlMode.PercentOutput, 0);
                } else {
                    mRotationFalcon.set(mRotationControlMode, mRotationDemand);
                }

                break;
        }

        var extensionDemand = mExtensionDemand;
        if (mAtRetractionLimit.getValue() && extensionDemand < 0) {
            extensionDemand = 0;
        }

        var shouldZero = mShouldZeroExtension.update(mAtRetractionLimit.getValue());
        if (shouldZero) {
            mExtensionFalcon.setSelectedSensorPosition(0);
        }

        switch (mArmExtensionState) {
            case STOP:
                mExtensionFalcon.set(ControlMode.PercentOutput, 0);
                break;
            case EXTENDED:
            case EXTENDING:
                // var shouldEngageBrake =
                //         mShouldEngageExtensionBrake.update(mArmExtensionState == ArmExtensionState.EXTENDED,
                // timestamp);
                // if (shouldEngageBrake) {
                //     mBrakes.engageExtensionBrake();
                // } else {
                //     mBrakes.disengageExtensionBrake();
                // }

                // if (mBrakes.extensionBrakeFinishedEngaging()) {
                //     mExtensionFalcon.set(ControlMode.PercentOutput, 0);
                // } else {
                //     mExtensionFalcon.set(mExtensionControlMode, extensionDemand);
                // }

                mExtensionFalcon.set(mExtensionControlMode, extensionDemand);
                break;
            default:
                break;
        }
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
        var measuredSetpoint = getMeasuredSetpoint();
        var demandSetpoint = getDemandSetpoint();
        var measuredPose = Constants.kArmKinematics.toPose(measuredSetpoint).getTranslation();
        var demandPose = Constants.kArmKinematics.toPose(demandSetpoint).getTranslation();

        logger.recordOutput("Arm/EncoderAngle", mPeriodicInputs.throughBoreEncoderDegrees);

        LoggingUtil.recordOutput("Arm/TargetPoint", mTargetPoint);
        LoggingUtil.recordOutput("Arm/MeasuredPoint", measuredPose); // Read this for arm Translation2d's
        LoggingUtil.recordOutput("Arm/DesiredSetpoint", mDesiredSetpoint);
        LoggingUtil.recordOutput("Arm/MeasuredSetpoint", measuredSetpoint);
        LoggingUtil.recordOutput("Arm/DemandSetpoint", demandPose);
        logger.recordOutput("Arm/ExtensionState", mArmExtensionState.toString());
        logger.recordOutput("Arm/RotationState", mArmRotationState.toString());
        logger.recordOutput("Arm/ControlState", mArmControlState.toString());
        logger.recordOutput("Arm/Retracted", mAtRetractionLimit.getValue());
        logger.recordOutput("Arm/Mechamism2d", mMechanism2d);
    }
}
