// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Telemetry;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteTrajectoryFollowerCommand extends Command {
  /** Creates a new RamseteTrajectoryFollowerCommand. */
  private DriveSubsystem mDrive;
  private CommandSwerveDrivetrain CSD = TunerConstants.DriveTrain;

  public RamseteTrajectoryFollowerCommand(DriveSubsystem mDrive) {
    this.mDrive = mDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  var thetaController =
        new ProfiledPIDController(
            TrajectoryConstants.kPThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            mDrive.TrajectoryGenerate(),
           Telemetry.m_lastPose, // Functional interface to feed supplier
            TunerConstants.SwerveConfig,

            // Position controllers
            new PIDController(TrajectoryConstants.kPXController, 0, 0),
            new PIDController(TrajectoryConstants.kPYController, 0, 0),
            thetaController,
            ,
            mDrive);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
