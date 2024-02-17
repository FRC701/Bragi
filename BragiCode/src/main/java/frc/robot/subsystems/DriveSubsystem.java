// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import java.util.List;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  // private SwerveDriveKinematicsConstraint SwerveTrajConfig;

  private SwerveDrivetrain mSwerveDrivetrain;

  private final SwerveModule fl;
  private final SwerveModule fr;
  private final SwerveModule bl;
  private final SwerveModule br;

  private TrajectoryConfig TrajConfig;

  private SwerveDriveOdometry m_odometry;

  public static PathConstraints mPathConstraints;
  public static HolonomicPathFollowerConfig mHoloConfig;
  public static ReplanningConfig mReplanningConfig = new ReplanningConfig(true, true);

  public static ChassisSpeeds mChassisSpeeds;

  private final SwerveDriveKinematics mSwerveDriveKinematics = TunerConstants.SwerveConfig;

  /*public static final ProfiledPIDController thetaController =
  new ProfiledPIDController(
      TrajectoryConstants.kPThetaController,
      0,
      0,
      TrajectoryConstants.kThetaControllerConstraints); */

  public DriveSubsystem() {

    AutoBuilder.configureHolonomic(
        this::Pose2d, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::GetSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
            4.79, // Max module speed, in m/s
            0.6096, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                mSwerveDrivetrain.getPigeon2().reset();

              } catch (Exception e) {

              }
            })
        .start();
    // SwerveTrajConfig = new SwerveDriveKinematicsConstraint(TunerConstants.SwerveConfig, 12);
    // mSwerveDrivetrain = new SwerveDrivetrain(TunerConstants.DrivetrainConstants,
    // TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
    // TunerConstants.BackRight);
    TrajConfig =
        new TrajectoryConfig(
                TrajectoryConstants.kMaxSpeedMetersPerSecond,
                TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(mSwerveDriveKinematics);

    mSwerveDrivetrain = TunerConstants.DriveTrain;

    fl = mSwerveDrivetrain.getModule(0);
    fr = mSwerveDrivetrain.getModule(1);
    bl = mSwerveDrivetrain.getModule(2);
    br = mSwerveDrivetrain.getModule(3);

    m_odometry =
        new SwerveDriveOdometry(
            mSwerveDriveKinematics, mSwerveDrivetrain.getPigeon2().getRotation2d(), GetPositons());

    mPathConstraints =
        new PathConstraints(
            TrajectoryConstants.kMaxSpeedMetersPerSecond,
            TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared,
            TrajectoryConstants.kMaxAngularSpeedRadiansPerSecond,
            TrajectoryConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    mHoloConfig =
        new HolonomicPathFollowerConfig(
            TrajectoryConstants.kMaxSpeedMetersPerSecond, 10.5, mReplanningConfig);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // mSwerveDrivetrain.seedFieldRelative(new Pose2d(0, 0, new Rotation2d()));
  }

  public PathPlannerPath TestTrajectory() {
    var wp4 = new GoalEndState(3, new Rotation2d(180));

    var wp1 = new Pose2d(0, 0, new Rotation2d(0));
    var wp2 = new Pose2d(1, 0, new Rotation2d(0));
    var wp3 = new Pose2d(2, 0, new Rotation2d(0));

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(wp1, wp2, wp3);

    PathPlannerPath mPath = new PathPlannerPath(bezierPoints, mPathConstraints, wp4);

    return mPath;
  }

  public Trajectory OldTrajectory() {
    var wp1 = new Pose2d(0, 0, new Rotation2d(0));
    var wp3 = new Pose2d(3, 3, new Rotation2d(2 * Math.PI));

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            wp1, List.of(new Translation2d(3, 0), new Translation2d(3, 0)), wp3, TrajConfig);

    return trajectory;
  }

  public Pose2d Pose2d() {
    return m_odometry.getPoseMeters();
  }

  public void SetDesiredStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, TrajectoryConstants.kMaxSpeedMetersPerSecond);
    fl.apply(desiredStates[0], DriveRequestType.Velocity);
    fr.apply(desiredStates[1], DriveRequestType.Velocity);
    bl.apply(desiredStates[2], DriveRequestType.Velocity);
    br.apply(desiredStates[3], DriveRequestType.Velocity);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(mSwerveDrivetrain.getPigeon2().getRotation2d(), GetPositons(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      fl.getCurrentState(), fr.getCurrentState(), bl.getCurrentState(), br.getCurrentState(),
    };
  }

  public ChassisSpeeds GetSpeeds() {
    return mSwerveDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = mSwerveDriveKinematics.toSwerveModuleStates(targetSpeeds);
    SetDesiredStates(targetStates);
  }

  public SwerveModulePosition[] GetPositons() {
    return new SwerveModulePosition[] {
      fl.getPosition(true), fr.getPosition(true), bl.getPosition(true), br.getPosition(true),
    };
  }

  @Override
  public void periodic() {

    m_odometry.update(mSwerveDrivetrain.getPigeon2().getRotation2d(), GetPositons());
    // AutoBuilder.followPath(TestTrajectory());

    // SmartDashboard.putString("ChassisSpeeds", mChassisSpeeds.toString());

    double[] currentOdomPose = {
      m_odometry.getPoseMeters().getX(),
      m_odometry.getPoseMeters().getY(),
      m_odometry.getPoseMeters().getRotation().getRadians()
    };
    SmartDashboard.putNumberArray("GetOdomPose", currentOdomPose);

    SmartDashboard.putNumber(
        "GyroHeading", -mSwerveDrivetrain.getPigeon2().getAngle() * (Math.PI / 180));

    /*
    SmartDashboard.putString(
        "mSwerveDrivetrain Mod", mSwerveDrivetrain.getModule(0).getPosition(true).toString());
    SmartDashboard.putString("SwerveModule", fl.getPosition(true).toString()); */
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("OdometryValid", mSwerveDrivetrain.odometryIsValid());

  }
}
