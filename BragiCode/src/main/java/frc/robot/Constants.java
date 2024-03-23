// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean kOpposeMasterDirection = true;
  public static final boolean kDontOpposeMasterDirection = false;
  public static final int kCANdleAddress = 9;
  public static final CANdle.LEDStripType kCANdleLEDStripType = LEDStripType.GRB;

  public static class IntakeConstants {
    public static final int kIntakeMotor = 28;
  }

  public static class FeederConstants {
    public static final int kFeederMotor = 24; // 24
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class ShooterConstants {
    public static final int kShooterMotorTop = 26; // 26
    public static final int kShooterMotorBottom = 25; // 25
    public static final double kShooterHeight = 11.5;
    public static final double kSpeakerHeight = 85;
    // Naming Convention kPt = Proportional Gain Top
    public static final double kPt = 0.1; // 0.765
    public static final double kIt = 0;
    public static final double kDt = 0;
    public static final double kVt = 0.125 - 0.0064; // 0.128
    public static final double kAt = 100;

    // Naming Convention kPb = Proportional Gain Bottom
    public static final double kPb = 0.1; // 1.3
    public static final double kIb = 0;
    public static final double kDb = 0;
    public static final double kVb = 0.125; // 1351
    public static final double kAb = 100;

    public static final double kShooterTopReduction = 1 / 1; // TopRollerGearRatios
    public static final double kShooterBottomReduction = 1 / 1; // BottomRollerGearRatios
  }

  public static class PivotConstants {
    public static final int kPivotMotor = 27;
    public static final int kThroughBoreChannel = 1;
    public static final double kEncoderToZero = 289.918218; // 34
    public static final double kPivotAngleMax = 62; // degrees
    public static final double kPivotAngleMin = 40; // degrees
    public static final double kEncoderOffset = 40;
    public static final double kEncoderRange = 62 - 40; //
    public static final double kEncoderUpperBound = 11.966130;
    public static final double kThroughBoreChannelMultiplier = 2.84444444444;

    public static final double kP = 0.8;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kV = 0;
    public static final double kG = 0;
    public static final double kS = 0;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorLeft = 29;
    public static final int kElevatorMotorRight = 30;
  }

  public static class IMUConstants {
    public static final String kGyroDeviceType = "Pigeon2";
    // public static final String kGyroDeviceType = "navX";
    public static final int kGyroDeviceNumber = 0;
  }

  /** Constants revolving around the vision subsystem. */
  public static final class VisionConstants {

    public static final Vector<N3> kStateStds =
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> kVisionStds =
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
    // Camera name
    // Must match camera set at photonvision.local:5800

    // Default
    // public static final String cameraName = "Camera_Module_v1";
    // public static final String cameraName = "photonvision";
    public static final String cameraName = "limelight_v3";
    // public static final String cameraName = "limelight_v2";

    // ---------- Vision
    // Constants about how your camera is mounted to the robot
    // public static final double CAMERA_PITCH_RADIANS =
    // Units.degreesToRadians(60); // Angle "up" from horizontal
    // public static final double CAMERA_HEIGHT_METERS = 0.1; // Height above floor

    // // How far from the target we want to be (need to be up against the note)
    // public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);

    // ----------
    public static final double kPoseAmbiguityThreshold = 0.2;
    // Camera mounted facing backward, pivoted 60 degrees from horizon, on back left
    // corner (14" off
    // center, 12" off center, 7.5" up)
    public static final double kTargetHeightMeters =
        Units.inchesToMeters(80.52); // center height of speaker is 80.52"
    // above the ground.
    public static final double kCenterSpeakerAboveCenterAprilTag = 23.39;
    public static final double kHeightSpeakerCenterMeters =
        Units.inchesToMeters(80.52); // height of center of speaker
    public static final double kCameraHeightMeters =
        Units.inchesToMeters(9); // height of camera above the ground is
    // ~9" Zach Wolf 3/10/2024,
    public static final double kCameraRobotRelativeX = Units.inchesToMeters(-11); //
    public static final double kCameraRobotRelativeY = Units.inchesToMeters(-6.5); //
    public static final double kCameraRobotRelativeZ = Units.inchesToMeters(9); //
    public static final double kCameraMountPitchAngle =
        Units.degreesToRadians(
            60); // public static final double kCameraMountAngle = Units.degreesToRadians(60);
    public static final double kCameraMountRollAngle = Units.degreesToRadians(0);
    public static final double kCameraMountYawAngle = Units.degreesToRadians(0);

    // Robot to camera transform
    // 3d
    public static final Transform3d robotToCam3d =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(kCameraRobotRelativeX),
                Units.inchesToMeters(kCameraRobotRelativeY),
                Units.inchesToMeters(kCameraRobotRelativeZ)),
            new Rotation3d(0.0, Units.degreesToRadians(kCameraMountPitchAngle), 0.0));
    // create 2d transform from 3d
    public static final Transform2d robotToCam2d =
        new Transform2d(
            robotToCam3d.getX(),
            Units.inchesToMeters(robotToCam3d.getY()),
            new Rotation2d(robotToCam3d.getRotation().getZ()));
  }

  public static class TrajectoryConstants {

    public static Pose2d bluetargetPoseAmp =
        new Pose2d(1.69, 6.66, new Rotation2d(Units.degreesToRadians(-143.59)));

    public static Pose2d redtargetPoseAmp =
        new Pose2d(14.85, 6.42, new Rotation2d(Units.degreesToRadians(-24.78)));

    public static Pose2d bluetargetSource =
        new Pose2d(15.34, 1.27, new Rotation2d(Units.degreesToRadians(-62.02)));

    public static Pose2d redtargetSource =
        new Pose2d(1.32, 1.28, new Rotation2d(Units.degreesToRadians(-121.29)));
    // Where one of the Blue Speaker Center AprilTag is located (#7)
    public static final Pose3d aprilTagPoseBlueSpeaker =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(-1.5),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(57.13)), // (center of vision target)
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));

    // Where one of the Blue Speaker Center AprilTag is located (#3)
    public static final Pose3d aprilTagPoseRedSpeaker =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(196.17),
                Units.inchesToMeters(57.13)), // (center of vision target)
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
    // Where one of the Red AprilTag is located (#5)

    public static final Pose3d aprilTagPoseRedAmp =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(578.77),
                Units.inchesToMeters(323),
                Units.inchesToMeters(53.38)), // (center of vision target)
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(270)));
    // Where one of the Blue AprilTag is located (#6)
    public static final Pose3d aprilTagPoseBlueAmp =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(72.5),
                Units.inchesToMeters(323),
                Units.inchesToMeters(53.38)), // (center of vision target)
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(270)));

    // above ground (field relative)
    // public static final double kHeightSpeakerCenterMeters =
    // Units.inchesToMeters(80.52)// height of center of speaker above ground (field
    // relative)
    public static final Pose3d Pose3dPoseBlueSpeakerCenter =
        aprilTagPoseBlueSpeaker.transformBy(
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(VisionConstants.kCenterSpeakerAboveCenterAprilTag),
                    Units.inchesToMeters(0)),
                new Rotation3d(0.0, 0, 0.0)));

    public static Pose2d aprilTagPoseBlueAmp2d = aprilTagPoseBlueAmp.toPose2d();
    public static Pose2d targetPoseBlueAmpFieldRelativePose2d =
        aprilTagPoseBlueAmp2d.transformBy(
            new Transform2d(Units.inchesToMeters(-16), 0, new Rotation2d(0)));

    public static Pose2d targetPoseAmp = new Pose2d(1, 1, new Rotation2d(180));

    public static final double kMaxSpeedMetersPerSecond = 4.73;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.73;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kPThetaController = 5;

    public static PathConstraints PathConstraint =
        new PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared,
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAccelerationMetersPerSecondSquared);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
