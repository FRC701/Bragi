// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.Telemetry;
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

  /*public static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          TrajectoryConstants.kPThetaController,
          0,
          0,
          TrajectoryConstants.kThetaControllerConstraints); */

  public DriveSubsystem() {

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
            .setKinematics(TunerConstants.SwerveConfig);

    mSwerveDrivetrain = TunerConstants.DriveTrain;

    fl = mSwerveDrivetrain.getModule(0);
    fr = mSwerveDrivetrain.getModule(1);
    bl = mSwerveDrivetrain.getModule(2);
    br = mSwerveDrivetrain.getModule(3);

    m_odometry =
        new SwerveDriveOdometry(
            TunerConstants.SwerveConfig,
            new Rotation2d(mSwerveDrivetrain.getPigeon2().getAngle()),
            new SwerveModulePosition[] {
              fl.getPosition(true),
              fr.getPosition(true),
              bl.getPosition(true),
              br.getPosition(true),
            });

    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //mSwerveDrivetrain.seedFieldRelative(new Pose2d(0, 0, new Rotation2d()));
  }

  public Trajectory TestTrajectory() {
    var wp1 = new Pose2d(0, 0, new Rotation2d(0));
    var wp3 = new Pose2d(3, 3, new Rotation2d(2* Math.PI));

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            wp1, List.of(new Translation2d(3, 0), new Translation2d(3,0 )), wp3, TrajConfig);
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
    m_odometry.resetPosition(
        Pose2d().getRotation(),
        new SwerveModulePosition[] {
          fl.getPosition(true), fr.getPosition(true), bl.getPosition(true), br.getPosition(true)
        },
        pose);
  }

  @Override
  public void periodic() {
    m_odometry.update(
        Pose2d().getRotation(),
        new SwerveModulePosition[] {
          fl.getPosition(true), fr.getPosition(true), bl.getPosition(true), br.getPosition(true)
        });

    double[] currentpose = {
      Telemetry.m_lastPose.getX(),
      Telemetry.m_lastPose.getY(),
      Telemetry.m_lastPose.getRotation().getRadians()
    };
    SmartDashboard.putNumberArray("GetTelemetryPose", currentpose);

    double[] currentOdomPose = {
      m_odometry.getPoseMeters().getX(),
      m_odometry.getPoseMeters().getY(),
      m_odometry.getPoseMeters().getRotation().getRadians()
    };
    SmartDashboard.putNumberArray("GetOdomPose", currentOdomPose);

    SmartDashboard.putNumber(
        "GyroHeading", -mSwerveDrivetrain.getPigeon2().getAngle() * (Math.PI / 180));

    SmartDashboard.putString(
        "mSwerveDrivetrain Mod", mSwerveDrivetrain.getModule(0).getPosition(true).toString());
    SmartDashboard.putString("SwerveModule", fl.getPosition(true).toString());
    // This method will be called once per scheduler run
   // SmartDashboard.putBoolean("OdometryValid", mSwerveDrivetrain.odometryIsValid());

  }
}
