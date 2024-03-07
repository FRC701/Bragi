// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.126;
  }

  public static class PivotConstants {
    public static final int kPivotMotor = 27;
    public static final int kThroughBoreChannel = 1;
    public static final double kEncoderToZero = 289.918218; // 34
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
    public static final int kElevatorMotorLeft = 0;
    public static final int kElevatorMotorRight = 0;
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
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    // Camera name
    // Must match camera set at photonvision.local:5800

    // Default
    // public static final String cameraName = "Camera_Module_v1";
    // public static final String cameraName = "photonvision";
    public static final String cameraName = "limelight_v3";
    // public static final String cameraName = "limelight_v2";

    // ---------- Vision
    // Constants about how your camera is mounted to the robot
    public static final double CAMERA_PITCH_RADIANS =
        Units.degreesToRadians(90); // Angle "up" from horizontal
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0); // Height above floor

    // How far from the target we want to be (need to be up against the note)
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);

    // Where one of the Blue Source AprilTags is located
    public static final Pose3d TARGET_POSE =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(593.68),
                Units.inchesToMeters(9.68),
                Units.inchesToMeters(53.38)), // (center of vision target)
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(120)));
    // ----------
    public static final double kTargetHeightMeters = Units.inchesToMeters(50);
    public static final double kCameraHeightMeters = Units.inchesToMeters(26);
    public static final double kCameraMountAngle = Units.degreesToRadians(45);
    public static final double y = 0;
    public static final double kCameraOffset = 0;
    // Robot to camera transform
    public static final Transform3d robotToCam3d =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-14), Units.inchesToMeters(-1.5), Units.inchesToMeters(7.5)),
            new Rotation3d(0.0, Units.degreesToRadians(45), 0.0));
    public static final Transform2d robotToCam2d =
        new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0.0, 0.0));

    // The difference in height between the target's height and the height of the
    // camera.
    public static final int deltaHeight = 0;
    public static final int cameraAngle = 90;

    // Camera mounted facing forward, half a meter forward of center, half a meter
    // up
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.5, 0.0, 0.0),
            new Rotation3d(
                0, 0, 0)); // 7 1/8 - 0.41" above board base + CS lab bench 34"= 37 1/8= 36 3/4"
  }

  public static class TrajectoryConstants {

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
