package com.team1701.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1701.frc2023.Constants;
import com.team1701.frc2023.loops.ILooper;
import com.team1701.frc2023.loops.Loop;
import com.team1701.lib.drivers.RetroreflectiveSensor;
import com.team1701.lib.drivers.TalonSRXFactory;
import com.team1701.lib.util.TimeLockedBoolean;
import com.team1701.lib.util.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends Subsystem {
    private static Intake mInstance = null;

    private IntakeState mIntakeStateBeforeShutoff = IntakeState.STOP;
    private IntakeState mIntakeState = IntakeState.STOW;
    private RollerState mRollerState = RollerState.STOP;
    private RotationState mRotationState = RotationState.STOP;
    private TimeLockedBoolean mIntakeHasPiece;
    private TimeLockedBoolean mCanDisengageRetain;
    private boolean mRotationMotorAtPosition;
    private double mAngle;

    private final TalonSRX mRotationMotorController;
    private final TalonSRX mRollerMotorController;
    private final DutyCycleEncoder mThroughBoreEncoder;
    private final PIDController mRotationPIDController;

    private double mRotationDemand = 0;
    private double mRollerDemand = 0;

    private RetroreflectiveSensor mRetroreflectiveSensor;

    private final IntakeInputsAutoLogged mIntakeInputs = new IntakeInputsAutoLogged();

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public Intake() {
        mThroughBoreEncoder = new DutyCycleEncoder(Constants.kIntakeThroughBoreEncoderID);
        mThroughBoreEncoder.setDistancePerRotation(360);

        mRotationMotorController = TalonSRXFactory.createTalon(Constants.kIntakeRotationMotorController, config -> {
            config.NEUTRAL_MODE = NeutralMode.Coast;
            config.openloopRamp = 0.05;
            config.continuousCurrentLimit = 5;
            config.nominalOutputForward = 0.15;
            config.nominalOutputReverse = -0.08;
            config.neutralDeadband = 0.02;
        });

        mRollerMotorController = TalonSRXFactory.createTalon(Constants.kIntakeRollerMotorController, config -> {
            config.NEUTRAL_MODE = NeutralMode.Coast;
            config.openloopRamp = 0.2;
        });

        mRotationPIDController = new PIDController(0.01, 0, 0);
        mRotationPIDController.setTolerance(2);

        mIntakeHasPiece = new TimeLockedBoolean(0.05, 0.0);
        mCanDisengageRetain = new TimeLockedBoolean(3.0, 0.0);

        mRetroreflectiveSensor = new RetroreflectiveSensor(Constants.kIntakeSensorID);
    }

    @AutoLog
    public static class IntakeInputs {
        public double rawAngle;
        public boolean sensorTripped;
    }

    public void toggleIntakeState() {
        if (mIntakeState != IntakeState.STOP) {
            mIntakeStateBeforeShutoff = mIntakeState;
            mIntakeState = IntakeState.STOP;
        } else {
            mIntakeState = mIntakeStateBeforeShutoff;
        }
    }

    public void setIntakeState(IntakeState state) {
        mIntakeState = state;
    }

    public IntakeState getIntakeState() {
        return mIntakeState;
    }

    public boolean atTarget() {
        return mRotationMotorAtPosition;
    }

    public boolean hasPiece() {
        return mIntakeHasPiece.getValue();
    }

    private void handleIntakeState() {
        // TODO: Set the motor to Brake when rejecting because it cannot maintain the point.
        switch (mIntakeState) {
            case HANDOFF_RELEASE:
                mRotationState = RotationState.RETRACTING;
                mRotationPIDController.setSetpoint(Constants.kIntakeUprightAngle);
                mRollerState = RollerState.HANDOFF;
                break;
            case HANDOFF_HOLD:
                mRotationState = RotationState.RETRACTING;
                mRotationPIDController.setSetpoint(Constants.kIntakeUprightAngle);
                mRollerState = RollerState.IN;
                break;
            case INTAKING:
                mRotationState = RotationState.EXTENDING;
                mRotationPIDController.setSetpoint(0.0);
                mRollerState = RollerState.IN;
                break;
            case REJECTING:
                mRotationPIDController.setSetpoint(45.0);
                mRotationState = RotationState.EXTENDING;
                mRollerState = RollerState.OUT;
                break;
            case STOP:
                mRotationState = RotationState.STOP;
                mRollerState = RollerState.STOP;
                break;
            case STOW:
                mRotationPIDController.setSetpoint(Constants.kIntakeUprightAngle);
                mRotationState = RotationState.RETRACTING;
                mRollerState = RollerState.IN;
                break;
            case STOW_NO_PIECE:
                mRotationPIDController.setSetpoint(Constants.kIntakeUprightAngle);
                mRotationState = RotationState.RETRACTING;
                mRollerState = RollerState.STOP;
                break;
            default:
                break;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRollerMotorController.setNeutralMode(NeutralMode.Brake);
            }

            @Override
            public void onLoop(double timestamp) {
                handleIntakeState();
                updateRotationDemand();
                updateRollerDemand();
            }

            @Override
            public String getDisplayName() {
                return Intake.class.getSimpleName();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private void updateRollerDemand() {
        NeutralMode rollerNeutralMode = NeutralMode.Brake;
        if (mRotationMotorAtPosition) {
            switch (mRollerState) {
                case HANDOFF:
                    rollerNeutralMode = NeutralMode.Coast;
                    mRollerDemand = 0.3;
                    break;
                case IN:
                    if (mIntakeHasPiece.getValue()) {
                        mRollerDemand = 0.0;
                        break;
                    }
                    if (mRotationState == RotationState.RETRACTING) {
                        if (mCanDisengageRetain.getValue()) {
                            mRollerDemand = 0.0;
                        } else {
                            mRollerDemand = -0.3;
                        }
                    } else {
                        mRollerDemand = -0.9;
                    }
                    break;
                case OUT:
                    mRollerDemand = 0.9;
                    break;
                default:
                    break;
            }
        } else {
            mRollerDemand = 0.0;
        }
        mRollerMotorController.setNeutralMode(rollerNeutralMode);
    }

    private void updateRotationDemand() {
        switch (mRotationState) {
            case STOP:
                mRotationDemand = 0.0;
                break;
            case EXTENDING:
            case RETRACTING:
                var pidControllerOutput = mRotationPIDController.calculate(mAngle);
                if (mRotationPIDController.atSetpoint()) {
                    mRotationDemand = 0.0;
                    break;
                }
                mRotationDemand = MathUtil.clamp(pidControllerOutput, -0.6, 0.6);
                break;
            default:
                break;
        }
    }

    @Override
    public void readPeriodicInputs() {

        mIntakeInputs.rawAngle = mThroughBoreEncoder.getDistance();
        mIntakeInputs.sensorTripped = mRetroreflectiveSensor.get();

        Logger.getInstance().processInputs("Intake", mIntakeInputs);

        mAngle = MathUtil.inputModulus(mIntakeInputs.rawAngle - Constants.kIntakeThroughBoreEncoderOffset, -180, 180);
        mRotationMotorAtPosition = Util.epsilonEquals(mRotationPIDController.getSetpoint(), mAngle, 10);

        mIntakeHasPiece.update(mIntakeInputs.sensorTripped, Timer.getFPGATimestamp());
        mCanDisengageRetain.update(!mIntakeInputs.sensorTripped, Timer.getFPGATimestamp());
    }

    @Override
    public void writePeriodicOutputs() {
        mRotationMotorController.set(ControlMode.PercentOutput, mRotationDemand);
        mRollerMotorController.set(ControlMode.PercentOutput, mRollerDemand);
    }

    @Override
    public void stop() {
        mRollerMotorController.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        var logger = Logger.getInstance();

        logger.recordOutput("Intake/AngleDegrees", mAngle);
        logger.recordOutput("Intake/Roller/State", mRollerState.toString());
        logger.recordOutput("Intake/Rotation/State", mRotationState.toString());
        logger.recordOutput("Intake/State", mIntakeState.toString());
        logger.recordOutput("Intake/HasPiece", mIntakeHasPiece.getValue());
        logger.recordOutput("Intake/Roller/Demand", mRollerDemand);
        logger.recordOutput("Intake/Rotation/Demand", mRotationDemand);
        logger.recordOutput("Intake/AtPosition", mRotationMotorAtPosition);
    }

    public enum IntakeState {
        HANDOFF_HOLD,
        HANDOFF_RELEASE,
        INTAKING,
        REJECTING,
        STOP,
        STOW,
        STOW_NO_PIECE,
        NONE
    }

    public enum RollerState {
        IN,
        OUT,
        STOP,
        HANDOFF
    }

    public enum RotationState {
        EXTENDING,
        RETRACTING,
        STOP
    }
}
