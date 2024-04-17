// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.commands.ActivateElevator;
import frc.robot.commands.EndAfterShot;
import frc.robot.commands.RevShooter;
import frc.robot.commands.SetVisionPivot;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SwitchPivotState;
import frc.robot.commands.ToggleAutoAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotEnumState;
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
  private double MaxAngularRate = TrajectoryConstants.kMaxAngularSpeedRadiansPerSecond * 0.75;
  private Feeder mFeeder = new Feeder();
  private ShooterSubsystem mShooter = new ShooterSubsystem();
  private Elevator mElevator = new Elevator();

  private Intake mIntake = new Intake();

  private VisionSubsystem mVisionSubsystem = new VisionSubsystem();

  private PivotSubsystem mPivotSubsystem = new PivotSubsystem();

  private LED mLed = new LED();

  private final CommandJoystick joystick =
      new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);

  private final CommandXboxController Driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController CODriver =
      new CommandXboxController(Constants.OperatorConstants.kCoDriverControllerPort); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Trigger TriggerJoystick = new Trigger(joystick.button(2));

  private final Trigger Button = new Trigger(Driver.leftStick());

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          // .withRotationalDeadband(MaxAngularRate * 0.28) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // AutoAim = Button.toggleOnTrue(null).getAsBoolean();

    // SmartDashboard.setDefaultNumber("Input Velocity", 0);
    SmartDashboard.setDefaultNumber("Input Angle", 0);

    Driver.x().onTrue(new SpinIntake(mIntake));
    Driver.a().onTrue(new Shoot(mShooter, 49.5));
    Driver.y().onTrue(new Shoot(mShooter, 20));
    Driver.b().onTrue(new RevShooter(mShooter, 49.5));
    // CODriver.povDownLeft().onTrue(new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_Fixed));
    // CODriver.a().onTrue(new Shoot(mShooter, 55 - 5.5));

    /*drivetrain.setDefaultCommand(
    drivetrain.applyRequest(
        () -> drive.withRotationalRate(mVisionSubsystem.TurnShooterToTargetOutput())));*/
    Driver.leftBumper().onTrue(new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_Fixed));
    Driver.rightBumper().onTrue(new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_VisionAim));

    Button.onTrue(new ToggleAutoAim());

    mElevator.setDefaultCommand(
        new ActivateElevator(
            mElevator,
            () ->
                (MathUtil.applyDeadband(
                    Driver.getLeftTriggerAxis() - Driver.getRightTriggerAxis(), 0.1))));

    /*mElevator.setDefaultCommand(
    new ActivateElevator(mElevator, () -> -CODriver.getRightTriggerAxis()));*/

    // drivetrain.setDefaultCommand(
    //     drivetrain.applyRequest(
    //         () ->
    // drive.withRotationalRate(Units.degreesToRadians(-mVisionSubsystem.TurnShooterToTargetOutput()))));

    // double RotOutput =
    //     ShooterSubsystem.AutoAim
    //         ? -joystick.getTwist() * MaxAngularRate
    //         : mVisionSubsystem.TurnShooterToTargetOutput();

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-Driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        ShooterSubsystem.AutoAim
                            ? (VisionSubsystem.HasTargets
                                ? MathUtil.applyDeadband(
                                    Units.degreesToRadians(
                                        -mVisionSubsystem.TurnShooterToTargetOutput()),
                                    0.05) // 0.05
                                : MathUtil.applyDeadband(
                                    -Driver.getRightX() * MaxAngularRate, MaxAngularRate * 0.35))
                            : MathUtil.applyDeadband(
                                -Driver.getRightX() * MaxAngularRate,
                                MaxAngularRate * 0.35)) // Drive counterclockwise with
            // negative X (left)
            ));

    // CODriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    Driver.rightStick()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

    // drivetrain.applyRequest(() -> drive.withRotationalRate(0));

    // reset the field-centric heading on left bumper press
    Driver.povCenter().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    NamedCommands.registerCommand("AutoAim", new ToggleAutoAim());
    NamedCommands.registerCommand("Shoot", new Shoot(mShooter, 55 - 5.5));
    NamedCommands.registerCommand("SetVisionPivot", new SetVisionPivot(mPivotSubsystem));
    NamedCommands.registerCommand(
        "SwitchPivotStateFixed", new SwitchPivotState(mPivotSubsystem, PivotEnumState.S_Fixed));
    NamedCommands.registerCommand("EndAfterShot", new EndAfterShot());

    NamedCommands.registerCommand("SpinIntake", new SpinIntake(mIntake));

    autoChooser = AutoBuilder.buildAutoChooser("AutoStraight");

    // autoChooser.addOption("Shoot", new Shoot(mShooter, 55 - 5.5));

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
