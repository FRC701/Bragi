// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTrajectoryFollower extends SwerveControllerCommand {
  @SuppressWarnings("unused")
  private DriveSubsystem mDrive;

  private static Trajectory mTrajectory;

  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          TrajectoryConstants.kPThetaController,
          0,
          0,
          TrajectoryConstants.kThetaControllerConstraints);

  /** Creates a new SwerveTrajectoryFollower. */
  public SwerveTrajectoryFollower(DriveSubsystem mDrive, Trajectory trajectory) {
    super(
        mTrajectory,
        mDrive::Pose2d, // Functional interface to feed supplier
        TunerConstants.SwerveConfig,

        // Position controllers
        new PIDController(TrajectoryConstants.kPXController, 0, 0),
        new PIDController(TrajectoryConstants.kPYController, 0, 0),
        thetaController,
        mDrive::SetDesiredStates,
        mDrive);

    this.mDrive = mDrive;
    mTrajectory = trajectory;

    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
