// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Generated.TunerConstants;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */


  private SwerveDriveKinematicsConstraint SwerveTrajConfig;

  private TrajectoryConfig TrajConfig;

  public DriveSubsystem() {
    SwerveTrajConfig = new SwerveDriveKinematicsConstraint(TunerConstants.SwerveConfig, 12);
    TrajConfig = new TrajectoryConfig(null, null);

    TrajConfig.addConstraint(SwerveTrajConfig);
  }

  public void TrajectoryGenerate()
  {
  var wp1 = new Pose2d();
  var wp3 = new Pose2d();

  var interWaypoints = new ArrayList<Translation2d>();
  interWaypoints.add(new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0)));

          var trajectory = TrajectoryGenerator.generateTrajectory(wp1, interWaypoints, wp3, TrajConfig);

  double duration = trajectory.getTotalTimeSeconds();


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
