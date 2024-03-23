package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  // public static boolean BlueTeam = false;
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final Field2d m_field = new Field2d();

  private VisionSubsystem m_vision = new VisionSubsystem();

  Optional<EstimatedRobotPose> GlobalVisionPose = m_vision.getEstimatedGlobalPose();

  private final CommandXboxController Driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          this.getPigeon2().getRotation2d(), // NWU
          new SwerveModulePosition[] {
            this.getModule(0).getPosition(true),
            this.getModule(1).getPosition(true),
            this.getModule(2).getPosition(true),
            this.getModule(3).getPosition(true)
          },
          new Pose2d(),
          VisionConstants.kStateStds,
          VisionConstants.kVisionStds);
  // ,
  // VisionConstants.kStateStds,
  // VisionConstants.kVisionStds
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
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
        // Change this if the path needs to be flipped on red vs blue
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void updateOdometry() {
    m_poseEstimator.update(
        this.getPigeon2().getRotation2d(), // NWU
        new SwerveModulePosition[] {
          this.getModule(0).getPosition(true),
          this.getModule(1).getPosition(true),
          this.getModule(2).getPosition(true),
          this.getModule(3).getPosition(true)
        });
    // Pose2d newEstimate = m_poseEstimator.getEstimatedPosition();
    // fieldPub.set(new double[] {
    //     newEstimate.getX(),
    //     newEstimate.getY(),
    //     newEstimate.getRotation().getDegrees()
    // });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on a real robot, this must be calculated based either on latency or
    // timestamps.

    // m_poseEstimator.addVisionMeasurement(
    // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    // m_poseEstimator.getEstimatedPosition()),
    // Timer.getFPGATimestamp() - 0.3);
    Optional<EstimatedRobotPose> GlobalVisionPose = m_vision.getEstimatedGlobalPose();
    if (GlobalVisionPose.isPresent()) {
      Pose2d pose2d =
          GlobalVisionPose.get().estimatedPose.toPose2d(); // this.setVisionMeasurementStdDevs();
      // this.setVisionMeasurementStdDevs(VisionConstants.kVisionStds);
      m_poseEstimator.addVisionMeasurement(pose2d, GlobalVisionPose.get().timestampSeconds - 0.3);
      // this.addVisionMeasurement(pose2d, GlobalVisionPose.get().timestampSeconds - 0.3);
      // m_poseEstimator.addVisionMeasurement(pose2d, GlobalVisionPose.get().timestampSeconds -
      // 0.3);
    }
  }

  public void dynamicallyChangeDeviations(Pose3d measurement, Pose2d currentEstimatedPose) {
    double dist =
        measurement.toPose2d().getTranslation().getDistance(currentEstimatedPose.getTranslation());
    double positionDev = Math.abs(0.2 * dist + 0.2);
    m_poseEstimator.setVisionMeasurementStdDevs(
        createStandardDeviations(positionDev, positionDev, Units.degreesToRadians(400)));
  }

  protected Vector<N3> createStandardDeviations(double x, double y, double z) {
    return VecBuilder.fill(x, y, z);
  }

  public boolean GetAllianceBlue() {
    var alliance = DriverStation.getAlliance();

    boolean OnBlue = false;

    if (alliance.isPresent()) {
      OnBlue = (alliance.get() == DriverStation.Alliance.Blue);
    }

    return OnBlue;
  }

  public Command PathToTarmacBlue() {
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            TrajectoryConstants.bluetargetPoseAmp,
            TrajectoryConstants.PathConstraint,
            0.0, // Goal end velocity in meters/sec
            0.0);

    return pathfindingCommand;
  }

  public Command PathToTarmacRed() {
    Command path =
        AutoBuilder.pathfindToPose(
            TrajectoryConstants.redtargetPoseAmp,
            TrajectoryConstants.PathConstraint,
            0.0, // Goal end velocity in meters/sec
            0.0);
    return path;
  }

  public Command PathToSourceBlue() {
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            TrajectoryConstants.bluetargetSource,
            TrajectoryConstants.PathConstraint,
            0.0, // Goal end velocity in meters/sec
            0.0);

    return pathfindingCommand;
  }

  public Command PathToSourceRed() {
    Command path =
        AutoBuilder.pathfindToPose(
            TrajectoryConstants.redtargetSource,
            TrajectoryConstants.PathConstraint,
            0.0, // Goal end velocity in meters/sec
            0.0);
    return path;
  }

  // public Command pathfindingCommand =
  //     new PathfindHolonomic(
  //         TrajectoryConstants.targetPoseAmp,
  //         TrajectoryConstants.PathConstraint,
  //         0.0,
  //         () -> this.getState().Pose,
  //         this::getCurrentRobotChassisSpeeds,
  //         (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
  //         new HolonomicPathFollowerConfig(
  //             new PIDConstants(10, 0, 0),
  //             new PIDConstants(10, 0, 0),
  //             TunerConstants.kSpeedAt12VoltsMps,
  //             10,
  //             new ReplanningConfig()),
  //         0.0,
  //         this);

  public void seedbackwards() {

    this.seedFieldRelative(
        new Pose2d(
            this.getState().Pose.getX(),
            this.getState().Pose.getY(),
            new Rotation2d(this.getState().Pose.getRotation().getRadians())));
  }

  @Override
  public void periodic() {
    // Driver.a().onTrue(GetAllianceBlue() ? PathToTarmacBlue() : PathToTarmacRed());
    // Driver.y().onTrue(GetAllianceBlue() ? PathToSourceBlue() : PathToSourceRed());

    // dynamicallyChangeDeviations(
    //     m_vision.hasTargets()
    //         ? m_vision.robotPose3dRelativeToField()
    //         : new Pose3d(this.getState().Pose),
    //     m_poseEstimator.getEstimatedPosition());
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putData("EstimationField", m_field);

    SmartDashboard.putString("Pose", m_poseEstimator.toString());
    SmartDashboard.putBoolean("ISpathCON", AutoBuilder.isPathfindingConfigured());
    SmartDashboard.putBoolean("ISCON", AutoBuilder.isConfigured());

    SmartDashboard.putBoolean("Team", GetAllianceBlue());

    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }
}
