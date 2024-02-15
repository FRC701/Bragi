// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.commands.Eject;
import frc.robot.commands.InputVelo;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.Stop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private double MaxSpeed =
      TrajectoryConstants.kMaxSpeedMetersPerSecond; // 6 meters per second desired top speed 6
  private double MaxAngularRate =
      0.75 * Math.PI; // 3/4 of a rotation per second max angular velocity 1.5 * pi

  private DriveSubsystem mDrive = new DriveSubsystem();
  private Feeder mFeeder = new Feeder();
  private ShooterSubsystem mShooter = new ShooterSubsystem();

  // private SwerveTrajectoryFollower mCommand = new SwerveTrajectoryFollower(mDrive,
  // mDrive.TestTrajectory());
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          TrajectoryConstants.kPThetaController,
          0,
          0,
          TrajectoryConstants.kThetaControllerConstraints);

  SwerveControllerCommand mSwerveControllerCommand =
      new SwerveControllerCommand(
          mDrive.OldTrajectory(),
          mDrive::Pose2d, // Functional interface to feed supplier
          TunerConstants.SwerveConfig,
          // Position controllers
          new PIDController(TrajectoryConstants.kPXController, 0, 0),
          new PIDController(TrajectoryConstants.kPYController, 0, 0),
          thetaController,
          mDrive::SetDesiredStates,
          mDrive);
  // PathfindThenFollowPathHolonomic mHolofollow = new
  // PathfindThenFollowPathHolonomic(mDrive.TestTrajectory(), DriveSubsystem.mPathConstraints ,
  // mDrive.Pose2d, mDrive.GetChassisSpeeds(), mDrive.SetDesiredStates(),
  // DriveSubsystem.mHoloConfig, false, mDrive);
  PathfindThenFollowPathHolonomic mFollowPathHolonomic =
      new PathfindThenFollowPathHolonomic(
          mDrive.TestTrajectory(),
          DriveSubsystem.mPathConstraints,
          mDrive::Pose2d,
          mDrive::GetSpeeds,
          mDrive::driveRobotRelative,
          DriveSubsystem.mHoloConfig,
          () -> false,
          mDrive);

  private final CommandJoystick joystick =
      new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController CODriver =
      new CommandXboxController(Constants.OperatorConstants.kCoDriverControllerPort); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Trigger TriggerJoystick = new Trigger(joystick.button(2));

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.28) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    PathPlannerPath straightPath = PathPlannerPath.fromPathFile("Straight Routine");

    SmartDashboard.putData("PPfollowStraightPath", AutoBuilder.followPath(straightPath));

              // The rotation component in these poses represents the direction of travel
              Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
              Pose2d endPos =
                  new Pose2d(
                      currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                      new Rotation2d());

              List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
              PathPlannerPath path =
                  new PathPlannerPath(
                      bezierPoints,
                      new PathConstraints(
                          4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                      new GoalEndState(0.0, currentPose.getRotation()));

              // Prevent this path from being flipped on the red alliance, since the given positions
              // are already correct
              path.preventFlipping = true;

              AutoBuilder.followPath(path).schedule();
            }));

      AutoBuilder.followPath(path).schedule();
    }));


    SmartDashboard.putData(
        "runTraj",
        Commands.sequence(
            new ResetOdometry(mDrive, mDrive.OldTrajectory()),mSwerveControllerCommand, new Stop()));

    SmartDashboard.setDefaultNumber("Input Velocity", 0);
    CODriver.a().onTrue(new InputVelo(mShooter));
    CODriver.y().onTrue(new Eject(mFeeder));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getY() * 0.25 * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -joystick.getX() * 0.25 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getTwist()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    CODriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    CODriver.b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))));

    // reset the field-centric heading on left bumper press
    TriggerJoystick.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
