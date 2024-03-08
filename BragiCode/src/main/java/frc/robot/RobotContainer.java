// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.commands.Eject;
import frc.robot.commands.InputVelo;
import frc.robot.commands.ReturnNormalState;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SwitchPivotState;
import frc.robot.commands.ToggleAutoAim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotEnumState;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = TrajectoryConstants.kMaxSpeedMetersPerSecond;
  private double MaxAngularRate = TrajectoryConstants.kMaxAngularSpeedRadiansPerSecond * 0.6;
  private Feeder mFeeder = new Feeder();
  private ShooterSubsystem mShooter = new ShooterSubsystem();
  private Elevator mElevator = new Elevator();

  private Intake mIntake = new Intake();

  private VisionSubsystem mVisionSubsystem = new VisionSubsystem();

  private PivotSubsystem mPivotSubsystem = new PivotSubsystem();

  private LED mLed = new LED();

  private final CommandJoystick joystick =
      new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController CODriver =
      new CommandXboxController(Constants.OperatorConstants.kCoDriverControllerPort); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Trigger TriggerJoystick = new Trigger(joystick.button(2));

  private final Trigger Button = new Trigger(joystick.button(5));

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          //.withRotationalDeadband(MaxAngularRate * 0.28) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // AutoAim = Button.toggleOnTrue(null).getAsBoolean();

    SmartDashboard.setDefaultNumber("Input Velocity", 0);
    SmartDashboard.setDefaultNumber("Input Angle", 0);

    CODriver.x().onTrue(new SpinIntake(mIntake));
    CODriver.a().onTrue(new InputVelo(mShooter));
    CODriver.y().onTrue(new Eject(mFeeder));
    CODriver.b().onTrue(new ReturnNormalState(mFeeder));
    // CODriver.leftBumper().onTrue(new InputPivot(mPivotSubsystem));

    /*drivetrain.setDefaultCommand(
    drivetrain.applyRequest(
        () -> drive.withRotationalRate(mVisionSubsystem.TurnShooterToTargetOutput())));*/
    CODriver.leftBumper()
        .onTrue(new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_AgainstSpeaker));
    CODriver.rightBumper()
        .onTrue(new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_VisionAim));

    Button.onTrue(new ToggleAutoAim());

    // mElevator.setDefaultCommand(
    //     new ActivateElevator(mElevator, () -> CODriver.getLeftTriggerAxis()));

    // mElevator.setDefaultCommand(
    //     new ActivateElevator(mElevator, () -> -CODriver.getRightTriggerAxis()));

    SmartDashboard.putData("path", drivetrain.PathToTarmac());
    // drivetrain.setDefaultCommand(
    //     drivetrain.applyRequest(
    //         () -> drive.withRotationalRate(Units.degreesToRadians(-mVisionSubsystem.TurnShooterToTargetOutput()))));

    // double RotOutput =
    //     ShooterSubsystem.AutoAim
    //         ? -joystick.getTwist() * MaxAngularRate
    //         : mVisionSubsystem.TurnShooterToTargetOutput();

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getY() * 0.25 * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -joystick.getX() * 0.25 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        ShooterSubsystem.AutoAim
                            ?  MathUtil.applyDeadband(-joystick.getTwist() * MaxAngularRate, MaxAngularRate * 0.28)
                            : Units.degreesToRadians(-mVisionSubsystem
                                .TurnShooterToTargetOutput())) // Drive counterclockwise with
            // negative X (left)
            ));

    // CODriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    CODriver.b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))));

    // drivetrain.applyRequest(() -> drive.withRotationalRate(0));

    // reset the field-centric heading on left bumper press
    TriggerJoystick.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("AutoStraight");

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
