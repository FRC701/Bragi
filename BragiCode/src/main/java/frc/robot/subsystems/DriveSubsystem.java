// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import java.util.ArrayList;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  // private SwerveDriveKinematicsConstraint SwerveTrajConfig;

  private SwerveDrivetrain mSwerveDrivetrain;

  private TrajectoryConfig TrajConfig;

  private SwerveDriveOdometry m_odometry;

  public DriveSubsystem() {
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

    m_odometry =
        new SwerveDriveOdometry(
            TunerConstants.SwerveConfig,
            mSwerveDrivetrain.getPigeon2().getRotation2d(),
            new SwerveModulePosition[] {
              mSwerveDrivetrain.getModule(0).getPosition(true),
              mSwerveDrivetrain.getModule(1).getPosition(true),
              mSwerveDrivetrain.getModule(2).getPosition(true),
              mSwerveDrivetrain.getModule(3).getPosition(true),
            });
  }

  public Trajectory TestTrajectory() {
    var wp1 = new Pose2d();
    var wp3 = new Pose2d();

    var interWaypoints = new ArrayList<Translation2d>();
    interWaypoints.add(new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0)));

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(wp1, interWaypoints, wp3, TrajConfig);

    return trajectory;
  }

  public Pose2d Pose2d() {
    return m_odometry.getPoseMeters();
  }

  public void SetDesiredStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, TrajectoryConstants.kMaxSpeedMetersPerSecond);
    mSwerveDrivetrain.getModule(0).apply(desiredStates[0], DriveRequestType.OpenLoopVoltage);
    mSwerveDrivetrain.getModule(1).apply(desiredStates[1], DriveRequestType.OpenLoopVoltage);
    mSwerveDrivetrain.getModule(2).apply(desiredStates[2], DriveRequestType.OpenLoopVoltage);
    mSwerveDrivetrain.getModule(3).apply(desiredStates[3], DriveRequestType.OpenLoopVoltage);
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        mSwerveDrivetrain.getPigeon2().getRotation2d(),
        new SwerveModulePosition[] {
          mSwerveDrivetrain.getModule(0).getPosition(true),
          mSwerveDrivetrain.getModule(1).getPosition(true),
          mSwerveDrivetrain.getModule(2).getPosition(true),
          mSwerveDrivetrain.getModule(3).getPosition(true)
        },
        pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
