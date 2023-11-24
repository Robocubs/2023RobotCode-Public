package com.team1701.frc2023;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1701.frc2023.subsystems.OperatorInterfaceClient.INTFACTION;
import com.team1701.frc2023.subsystems.Superstructure;
import com.team1701.lib.arm.ArmKinematics;
import com.team1701.lib.arm.ArmSetpointGenerator.PassDirection;
import com.team1701.lib.swerve.ExtendedSwerveDriveKinematics;
import com.team1701.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team1701.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Constants {
    public static final double kLooperDt = 0.01;
    public static final double kRobotWidthWithBumpers = 0.832;
    public static final double kRobotWidth = 0.665;
    public static final double kRobotLengthWithBumpers = 0.978;
    public static final double kRobotLength = 0.813;
    public static final double kLineToTag = 0.35;
    public static final Map<String, INTFACTION> kActionMap = Map.ofEntries(
            Map.entry("subleft", INTFACTION.SUBLEFT),
            Map.entry("subright", INTFACTION.SUBRIGHT),
            Map.entry("intakeextend", INTFACTION.INTAKEEXTEND),
            Map.entry("intakeretract", INTFACTION.INTAKERETRACT),
            Map.entry("intakereject", INTFACTION.INTAKEREJECT),
            Map.entry("clawtoggle", INTFACTION.CLAWTOGGLE),
            Map.entry("intaketoggle", INTFACTION.INTAKETOGGLE),
            Map.entry("armhome", INTFACTION.ARMHOME),
            Map.entry("pos", INTFACTION.POSITION),
            Map.entry("armdunk", INTFACTION.DUNK),
            Map.entry("compress", INTFACTION.COMPRESS));

    /* START FIELD CONSTANTS */
    public static final double kFieldWidthMeters = 16.540988;
    // Deli meter, the SI unit for measuring meat
    public static final double kBlueCommunityDelimeterX = 4.0;
    public static final double kRedCommunityDelimeterX = 12.8;
    public static final double kBlueCommunityDelimeterY = 5.5;
    public static final double kRedCommunityDelimeterY = kBlueCommunityDelimeterY;
    public static final int kBlueAprilTagIds[] = {8, 7, 6};
    public static final int kRedAprilTagIds[] = {3, 2, 1};
    /* END FIELD CONSTANTS */

    /* START INTAKE CONSTANTS */
    public static final double kIntakeUprightAngle = 87.0;
    public static final int kIntakeRotationMotorController = 43;
    public static final int kIntakeRollerMotorController = 42;
    public static final int kIntakeThroughBoreEncoderID = 5;
    public static final double kIntakeThroughBoreEncoderOffset = 300.7;

    public static final int kIntakeSensorID = 4;
    /* END INTAKE CONSTANTS */

    /* START ARM/HAND CONSTANTS */
    public static final double kHandReachedSetpointBounceDuration = 0.1;
    public static final double kArmThroughBoreEncoderOffset = 28.345;
    public static final Pose2d kArmPosition = new Pose2d(0.03, 1.05, GeometryUtil.kRotationHalfPi.unaryMinus());
    public static final Translation2d kHandPosition = new Translation2d(0.695, -0.05);

    public static final double kFalconMaxVelocity = 6380.0 / 60.0 / 10.0 * 2048.0;
    public static final double kArmRotationCruiseVelocity = kFalconMaxVelocity / 1.0;
    public static final double kArmRotationAcceleration = kArmRotationCruiseVelocity / 0.2;
    public static final ControlMode kArmRotationClosedLoopControlMode = ControlMode.MotionMagic;

    public static final PassDirection kArmPassDirection = PassDirection.OVER;
    public static final double kHandActuationTime = 0.5;
    public static final double kBrakeActuationTime = 0.1;

    public static final double kArmExtensionLimit =
            Units.feetToMeters(4) + kRobotLength / 2 - kArmPosition.getX() - kHandPosition.getX();
    public static final double kRotationTalonReduction = 4 * 4 * 4 * (30.0 / 50.0) * 4;
    public static final double kExtensionTalonReduction = 10.61 / 1;
    public static final double kSpoolDiameterInches = 1.2;

    public static final int kPneumaticsHubId = 30;
    public static final int kHandCloseId = 3;
    public static final int kHandReleaseId = 2;

    public static final int kArmRotationBrakeEngageId = 1;
    public static final int kArmRotationBrakeDisengageId = 0;

    public static final int kArmExtensionBrakeEngageId = 4;
    public static final int kArmExtensionBrakeDisengageId = 5;

    public static final int kCompressorMinPressure = 70;
    public static final int kCompressorMaxPressure = 120;

    public static final int kArmRotationTalonID = 20;
    public static final int kArmExtensionTalonID = 51;

    public static final int kArmThroughBoreEncoderID = 9;
    public static final int kArmRetractionSensorID = 7;
    public static final int kHandSensorID = 8;

    public static final Rotation2d kAngleToFront = Rotation2d.fromDegrees(37);
    public static final Rotation2d kAngleToBack = Rotation2d.fromDegrees(-45);

    public static final Rotation2d kArmMeasuredRotationErrorMargin = Rotation2d.fromDegrees(2);
    public static final double kArmMeasuredExtensionErrorMargin = 0.01;

    public static final Rotation2d kMinArmRotation = kArmPassDirection == PassDirection.THROUGH
            ? Rotation2d.fromDegrees(20)
            : kAngleToFront.plus(kArmMeasuredRotationErrorMargin);
    public static final Rotation2d kMaxArmRotation = kArmPassDirection == PassDirection.THROUGH
            ? Rotation2d.fromDegrees(170)
            : kAngleToBack.minus(kArmMeasuredRotationErrorMargin);

    public static final ArmKinematics kArmKinematics = new ArmKinematics(kArmPosition, kHandPosition);

    public static final double kHandLength = 0.14;
    public static final double kMaxExtension = Units.feetToMeters(4) + kRobotLength / 2 + 0.05;
    public static final double kMaxHeight = Units.feetToMeters(6.5);
    public static final double kMaxArmXPosition = kMaxExtension - kArmPosition.getX();
    public static final double kMaxArmYPosition = kMaxHeight - kArmPosition.getY();

    private static final Translation2d kArmHoldPosition = kArmPassDirection == PassDirection.THROUGH
            ? new Translation2d(kRobotLength / 2 - 0.1, 0.3)
            : new Translation2d(kArmPosition.getX() - kHandPosition.getY(), kArmPosition.getY() + kHandPosition.getX());

    public static final Map<Superstructure.TargetType, Translation2d> kArmSetpoints = Map.ofEntries(
            Map.entry(Superstructure.TargetType.HIGH_CONE, new Translation2d(kMaxArmXPosition, 1.72)),
            Map.entry(Superstructure.TargetType.HIGH_CONE_FLOOR, new Translation2d(kMaxArmXPosition, 1.65)),
            Map.entry(Superstructure.TargetType.HIGH_CUBE, new Translation2d(kMaxArmXPosition, 1.5)),
            Map.entry(Superstructure.TargetType.HIGH_CUBE_REVERSE, new Translation2d(-kMaxArmXPosition, 1.6)),
            Map.entry(Superstructure.TargetType.HIGH_CONE_REVERSE, new Translation2d(-kMaxArmXPosition, 2.05)),
            Map.entry(
                    Superstructure.TargetType.MID_CONE,
                    new Translation2d(
                            kRobotLengthWithBumpers / 2 + 0.58 + kHandLength / 2 - kArmPosition.getX(), 1.25)),
            Map.entry(
                    Superstructure.TargetType.MID_CUBE,
                    new Translation2d(kRobotLengthWithBumpers / 2 + 0.58 + kHandLength / 2 - kArmPosition.getX(), 1.1)),
            Map.entry(Superstructure.TargetType.MID_CUBE_REVERSE, new Translation2d(-1.12, 1.09)),
            Map.entry(
                    Superstructure.TargetType.FLOOR_PICKUP,
                    new Translation2d(kRobotLengthWithBumpers / 2 + 0.85, 0.25)),
            Map.entry(
                    Superstructure.TargetType.SUBSTATION, new Translation2d(kRobotLengthWithBumpers / 2 + 0.25, 1.13)),
            Map.entry(Superstructure.TargetType.HOLD, kArmHoldPosition),
            Map.entry(Superstructure.TargetType.HANDOFF_READY, new Translation2d(0.63, 0.69)),
            Map.entry(Superstructure.TargetType.HANDOFF_GRAB, new Translation2d(0.44, 0.46)),
            Map.entry(Superstructure.TargetType.LOW, new Translation2d(0.77, 0.49)));
    /* END ARM/HAND CONSTANTS */

    /* START CONTROLLER CONSTANTS */
    public static final double kDefaultDeadZone = 0.16;
    public static final double kDriverXDeadZone = 0.09;
    public static final double kDriverYDeadZone = 0.09;
    public static final int kDriverTurboBoostBumper = ControllerManager.kXBOXTriggerRight;
    public static final int kDriverSwerveLockButton = ControllerManager.kXBOXTriggerLeft;
    public static final int kDriverAllowOperatorAction = ControllerManager.kXBOXButtonY;
    public static final int kDriverSlowModeButton = ControllerManager.kXBOXBumperRight;
    /* END CONTROLLER CONSTANTS */

    /* START DRIVE CONSTANTS */
    public static final double kMaxAutonomousVelocity = 3.5;
    public static final double kMaxAutonomousAcceleration = 2.0;

    public static final double kMinLockSpeed = 0.2;
    public static final double kMinDriveSpeed = 0.2;

    /* Measure the drivetrain's maximum velocity or calculate the theoretical.
    The formula for calculating the theoretical maximum velocity is:
    <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    Theoretical: 4.16808
    TODO measure this */
    public static final double kMaxAngularVelocityRadiansPerSecond = 8.561212782;
    //     public static final double kMaxVelocityMetersPerSecond = 3.729073684;
    public static final double kMaxVelocityMetersPerSecond = 4.0;

    // Gear ratios
    public static final double kDriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    public static final double kSteerReduction = 1 / 12.8;

    public static final double kDriveWheelDiameter = 0.1016;
    public static final double kDriveTrackWidthMeters = 0.4953;
    public static final double kDriveWheelbaseMeters = 0.6477;

    public static final double kMaxDriveVoltage = 12.0;

    // Drive Falcon PIDF
    public static final double kMk4DriveVelocityKp = 0.1; // 0.1
    public static final double kMk4DriveVelocityKi = 0.0;
    public static final double kMk4DriveVelocityKd = 0.01; // 0.01
    public static final double kMk4DriveVelocityKf = 1023
            / (kMaxVelocityMetersPerSecond
                    / (Math.PI * Constants.kDriveWheelDiameter * Constants.kDriveReduction / 2048.0 * 10));

    // Steer Falcon PID
    public static final double kMk4AziKp = 0.75; // 0.75
    public static final double kMk4AziKi = 0;
    public static final double kMk4AziKd = 15; // 15
    public static final double kMaxAutonDistance = Units.feetToMeters(3);

    /* END DRIVE CONSTANTS */

    /* START DRIVE KINEMATIC LIMITS */
    public static final KinematicLimits kSlowModeKinematicLimits = new KinematicLimits();

    static {
        kSlowModeKinematicLimits.kMaxDriveVelocity = 1.0;
        kSlowModeKinematicLimits.kMaxDriveAcceleration = kSlowModeKinematicLimits.kMaxDriveAcceleration / 0.2;
        kSlowModeKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(90.0);
    }

    public static final KinematicLimits kSmoothKinematicLimits = new KinematicLimits();

    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * 0.7;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveVelocity / 0.6;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    public static final KinematicLimits kSlowKinematicLimits = new KinematicLimits();

    static {
        kSlowKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond * 0.1;
        kSlowKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveAcceleration;
        kSlowKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    public static final KinematicLimits kFastKinematicLimits = new KinematicLimits();

    static {
        kFastKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.4;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }

    public static final KinematicLimits kAutonomousKinematicLimits = new KinematicLimits();

    static {
        kAutonomousKinematicLimits.kMaxDriveVelocity = Constants.kMaxVelocityMetersPerSecond;
        kAutonomousKinematicLimits.kMaxDriveAcceleration = kAutonomousKinematicLimits.kMaxDriveVelocity / 0.2;
        kAutonomousKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }

    public static final boolean kPracticeBot = true;
    public static final ExtendedSwerveDriveKinematics kKinematics = new ExtendedSwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.kDriveTrackWidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(Constants.kDriveTrackWidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-Constants.kDriveTrackWidthMeters / 2.0, Constants.kDriveWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-Constants.kDriveTrackWidthMeters / 2.0, -Constants.kDriveWheelbaseMeters / 2.0));
    /* END DRIVE KINEMATIC LIMITS */

    /* START CANBUS CONFIG */
    public static final int kLongCANTimeoutMs = 100;
    /* END CANBUS CONFIG */

    /* START SWERVE MODULE CONFIG */
    public static final int kPigeonIMUId = 25;

    // FRONT LEFT MODULE
    public static final int kFrontLeftDriveTalonId = 4;
    public static final int kFrontLeftAziTalonId = 5;
    public static final Rotation2d kFrontLeftAziEncoderOffset = Rotation2d.fromDegrees(328.49);
    public static final int kFrontLeftEncoderPortId = 6;

    // FRONT RIGHT MODULE
    public static final int kFrontRightDriveTalonId = 14;
    public static final int kFrontRightAziTalonId = 13;
    public static final Rotation2d kFrontRightAziEncoderOffset = Rotation2d.fromDegrees(283.29);
    public static final int kFrontRightEncoderPortId = 15;

    // BACK LEFT MODULE
    public static final int kBackLeftDriveTalonId = 11;
    public static final int kBackLeftAziTalonId = 10;
    public static final Rotation2d kBackLeftAziEncoderOffset = Rotation2d.fromDegrees(232.95);
    public static final int kBackLeftEncoderPortId = 12;

    // BACK RIGHT MODULE
    public static final int kBackRightDriveTalonId = 1;
    public static final int kBackRightAziTalonId = 2;
    public static final Rotation2d kBackRightAziEncoderOffset = Rotation2d.fromDegrees(281.60);
    public static final int kBackRightEncoderPortId = 3;
    /* END SWERVE MODULE CONFIG */

    /* BEGIN VISION */
    public static final String kLimelightName = "limelight";
    public static final Transform3d kRobotToLimelightPose =
            new Transform3d(new Translation3d(0.225, 0.28, 0.455), new Rotation3d(0, 0, Units.degreesToRadians(-2)));
    public static final double kCubeGamePieceHeight = 0.27;

    public static final String kFrontCameraName = "Camera";
    public static final Transform3d kRobotToFrontCamPose =
            new Transform3d(new Translation3d(0.19, -0.30, 0.5), new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final String kBackCameraName = "Camera2";
    public static final Transform3d kRobotToBackCamPose =
            new Transform3d(new Translation3d(-0.195, 0.305, 0.5), new Rotation3d(0, 0, Units.degreesToRadians(180.5)));

    public static final double kCameraMaxPoseAmbiguity = 0.03;
    public static final double kCameraZoomMultiplier = 0.9531350826;
    public static final PoseStrategy kCameraPoseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
    /* END VISION */
}
