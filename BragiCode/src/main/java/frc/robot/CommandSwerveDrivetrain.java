package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  DoubleArrayPublisher fieldPub;

  private final Vision m_vision = new Vision();
  // Pigeon2 m_gyro = new Pigeon2(Constants.IMUConstants.kGyroDeviceNumber); // Pigeon is on CAN 1
  Pigeon2 m_gyro = new Pigeon2(1); // Pigeon is on CAN 1

  // private SwerveDrivetrain m_drive = TunerConstants.DriveTrain;
  SwerveModule m_frontLeft = TunerConstants.DriveTrain.getModule(0);
  SwerveModule m_frontRight = TunerConstants.DriveTrain.getModule(1);
  SwerveModule m_backLeft = TunerConstants.DriveTrain.getModule(2);
  SwerveModule m_backRight = TunerConstants.DriveTrain.getModule(3);
  private boolean refreshSwerevePositon = true;
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(), // NWU
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(refreshSwerevePositon),
            m_frontRight.getPosition(refreshSwerevePositon),
            m_backLeft.getPosition(refreshSwerevePositon),
            m_backRight.getPosition(refreshSwerevePositon)
          },
          new Pose2d());
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  StringPublisher fieldTypePub;

  public void Drivetrain() {
    m_gyro.reset();
    inst.startClient4("blarg");
    NetworkTable fieldTable = inst.getTable("field");
    fieldPub = fieldTable.getDoubleArrayTopic("robotPose").publish();
    fieldTypePub = fieldTable.getStringTopic(".type").publish();
    fieldTypePub.set("Field2d");
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    refreshSwerevePositon = true;
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(), // NWU
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(refreshSwerevePositon),
          m_frontRight.getPosition(refreshSwerevePositon),
          m_backLeft.getPosition(refreshSwerevePositon),
          m_backRight.getPosition(refreshSwerevePositon)
        },
        pose);

    Pose2d newEstimate = m_poseEstimator.getEstimatedPosition();
    fieldPub.set(
        new double[] {
          newEstimate.getX(), newEstimate.getY(), newEstimate.getRotation().getDegrees()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on a real robot, this must be calculated based either on latency or
    // timestamps.
    Optional<EstimatedRobotPose> m_visionPose = m_vision.getEstimatedGlobalPose();
    if (m_visionPose.isPresent()) {
      Pose2d m_visionPosePose2d = m_visionPose.get().estimatedPose.toPose2d();
      m_poseEstimator.addVisionMeasurement(
          m_visionPosePose2d, m_visionPose.get().timestampSeconds - 0.3);
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
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
}
