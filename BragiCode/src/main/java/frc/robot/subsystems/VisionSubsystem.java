// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Generated.TunerConstants;
import frc.robot.utils.limelight.FieldLayout;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// Ignore unused variable warnings
@SuppressWarnings("unused")
public class VisionSubsystem extends SubsystemBase {

  // vars
  public static boolean HasTargets = false;
  private Pose3d m_FieldToRobotAprilTagPose;
  // private Pose2d m_fieldRobotPose;
  private Pose3d m_RobotPose3d;
  private boolean m_FieldToRobotAprilTagPoseNull = true;
  private boolean m_fieldRobotPoseNull = true;
  private boolean m_RobotPose3dNull = true;
  private boolean m_RobotToCam3dNull = true;
  private String mCameraToTargetString;
  private String m_cameraToTargetTranslationString;
  private double distanceToTarget = 0;
  private Pose2d m_AprilTagTargetPose2d;
  private Pose3d m_AprilTagTargetPose3d;
  private Pose3d robotPose3dRelativeToField;
  private float dummyDouble = -99;

  private double MaxSpeed = TrajectoryConstants.kMaxSpeedMetersPerSecond;
  private double MaxAngularRate = TrajectoryConstants.kMaxAngularSpeedRadiansPerSecond;

  private static double pivotAngle = 100000;

  // public PhotonPoseEstimator photonPoseEstimator;
  // public AprilTagFieldLayout atfl;
  private final Field2d m_field = new Field2d();
  final double ANGULAR_P = 1.5; // 1.5
  final double ANGULAR_D = 0.01; // 0.01
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  SimpleMotorFeedforward turnfeed = new SimpleMotorFeedforward(2, 0, 0);

  final double PIVOT_P = 0.1;
  final double PIVOT_D = 0.0;
  PIDController pivotController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  // private final Joystick joystick = new
  // Joystick(Constants.OperatorConstants.kDriverControllerPort);

  private final CommandJoystick joystick =
      new CommandJoystick(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.28) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  XboxController xboxController = new XboxController(0);
  // Create a vision photon camera
  PhotonCamera mVisionCamera;
  private double lastEstTimestamp = 0;
  // Camera result for vision camera
  private PhotonPipelineResult mCameraResult;
  // Pose estimator
  private PhotonPoseEstimator mPoseEstimator;
  // read in Cam to robot transform
  private Transform3d m_robotToCamTransform3d = Constants.VisionConstants.robotToCam3d;
  // 2d version
  private Transform2d m_robotToCamTransform2d = Constants.VisionConstants.robotToCam2d;
  // get entire apriltag layout
  private AprilTagFieldLayout mAprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  // get pose of specific april tag from java file

  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  Pigeon2 pigeon = new Pigeon2(Constants.IMUConstants.kGyroDeviceNumber); // Pigeon is on CAN

  // Subsystem Constructor
  public VisionSubsystem() {
    // visionCamera.setVersionCheckEnabled(false);
    // visionCamera.setDriverMode(true);
    // Port forward photon vision so we can access it with an ethernet cable
    // PortForwarder.add(5800, "photonvision.local", 5800);
    // update the gyro if need be
    mVisionCamera = new PhotonCamera(Constants.VisionConstants.cameraName);

    mPoseEstimator =
        new PhotonPoseEstimator(
            mAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            mVisionCamera,
            VisionConstants.robotToCam3d);
    mPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    if (Constants.IMUConstants.kGyroDeviceType == "navX") {
      AHRS ahrs = new AHRS(SPI.Port.kMXP);
    } else {
      Pigeon2 pigeon = new Pigeon2(Constants.IMUConstants.kGyroDeviceNumber); // Pigeon is on CAN
      // Bus with device ID 0
    }

    // Update camera results before periodic
    updateCameraResults();
    mVisionCamera.setLED(VisionLEDMode.kDefault);

    SmartDashboard.putData("Field", m_field); // Do this in either robot or
    pivotController.setTolerance(0);
    turnController.setTolerance(0);

    // subsystem init
  }

  // Update the camera results
  private void updateCameraResults() {
    mCameraResult = mVisionCamera.getLatestResult();
  }

  // Checks if camera sees targets, must use to stop null exceptions!
  public Boolean hasTargets() {
    return (mCameraResult.hasTargets());
  }

  private void updatePoses() {
    if (hasTargets()) {
      // object/fiducial tag space (X forward, Y left, Z up)
      // define 2D pose of aprilTag
      // get best target April Tag ID
      int AprilTagID = getTargetID(); // OK
      // AprilTagID=16;
      Pose3d m_AprilTagTargetPose3d = FieldLayout.aprilTags.get(AprilTagID); // Blue Speaker (left)
      Pose2d m_AprilTagTargetPose2d = m_AprilTagTargetPose3d.toPose2d(); // OK

      Transform3d m_CameraToTargetTransform3d = getTargetTransform(); // OK
      // Estimates the pose of the robot in the field coordinate system, given the
      // pose of the fiducial tag, the robot relative to the camera, and the target
      // relative to the camera.

      // Calculate robot's field relative pose

      Pose3d robotPose3dRelativeToField =
          PhotonUtils.estimateFieldToRobotAprilTag(
              m_CameraToTargetTransform3d,
              m_AprilTagTargetPose3d,
              m_robotToCamTransform3d); // Not OK
      // calculate distance to target

      double distanceToTarget =
          PhotonUtils.getDistanceToPose(
              robotPose3dRelativeToField.toPose2d(), m_AprilTagTargetPose2d); // OK

      // Do this in either robot periodic or subsystem periodic
      m_field.setRobotPose(robotPose3dRelativeToField.toPose2d());
    }
  }

  public Pose3d robotPose3dRelativeToField() {
    Pose3d robotPose3dRelativeToField =
        PhotonUtils.estimateFieldToRobotAprilTag(
            getTargetTransform(), m_AprilTagTargetPose3d, m_robotToCamTransform3d);
    return robotPose3dRelativeToField;
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = mPoseEstimator.update();
    double latestTimestamp = mVisionCamera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    // if (Robot.isSimulation()) {
    //     visionEst.ifPresentOrElse(
    //             est ->
    //                     getSimDebugField()
    //                             .getObject("VisionEstimation")
    //                             .setPose(est.estimatedPose.toPose2d()),
    //             () -> {
    //                 if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
    //             });
    // }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public double GetTranslationDistance() {
    double dist =
        mCameraResult
            .getBestTarget()
            .getBestCameraToTarget()
            .getTranslation()
            .getDistance(
                mAprilTagFieldLayout
                    .getTagPose(mCameraResult.getBestTarget().getFiducialId())
                    .get()
                    .getTranslation());
    return dist;
  }

  // public Pose3d getPose3d(){
  //   return mCameraResult.getBestTarget().getBestCameraToTarget();
  // }
  // Returns the single best target from the camera
  private PhotonTrackedTarget getBestTarget() {
    return (mCameraResult.getBestTarget());
  }

  // Returns all targets from the camera in an array
  private List<PhotonTrackedTarget> getTargetList() {
    return (mCameraResult.getTargets());
  }

  // Returns a picked target from the target list
  private PhotonTrackedTarget getTargetFromList(int num) {
    return (mCameraResult.getTargets().get(num));
  }

  // Returns the size of the target list
  private int getListSize() {
    return (mCameraResult.getTargets().size());
  }

  // Returns a percentage of how much area a target takes up, 0 - 100 percent
  private double getTargetArea() {
    return (getBestTarget().getArea());
  }

  private double getTargetPitch() {
    return (getBestTarget().getPitch());
  }

  private double getTargetYaw() {
    return (getBestTarget().getYaw());
  }

  private double getTargetYaw7() {
    double Yaw = 0;

    if (hasTargets()) {
      List<PhotonTrackedTarget> targets = mCameraResult.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == 7) {
          Yaw = target.getYaw();
          break;
        }
        if (target.getFiducialId() == 4) {
          Yaw = target.getYaw();
        }
      }

      if (Yaw == 0) {
        mCameraResult.getBestTarget().getYaw();
      }
    }
    return Yaw;
  }

  // Returns the april tag ID number
  public int getTargetID() {
    return (getBestTarget().getFiducialId());
  }

  private double getTargetDistance() {
    return getTargetTransform().getX();
  }

  public Transform3d getTargetTransform() {
    return (getBestTarget().getBestCameraToTarget());
  }

  public double getTargetTransformHeight() {
    return (getBestTarget().getBestCameraToTarget().getZ());
  }

  public double getYaw() {
    if (Constants.IMUConstants.kGyroDeviceType == "navX") {
      if (ahrs.isConnected()) {
        return ahrs.getYaw();
      } else {
        return dummyDouble;
      }
    } else {
      return pigeon.getYaw().getValueAsDouble();
    }
  }

  // public double getPitch() {
  //   if (Constants.IMUConstants.kGyroDeviceType == "navX") {
  //     if (ahrs.isConnected()) {
  //       return ahrs.getPitch();
  //     } else {
  //       return dummyDouble;
  //     }
  //   } else {
  //     return pigeon.getPitch().getValueAsDouble();
  //   }
  // }

  public double getDistance() {
    double distance;
    if (mCameraResult.hasTargets()) {
      distance =
          (Constants.VisionConstants.kTargetHeightMeters
                  - Constants.VisionConstants.kCameraHeightMeters)
              / Math.tan(
                  Constants.VisionConstants.kCameraMountPitchAngle
                      + mCameraResult.getBestTarget().getPitch());
    } else {
      distance = 0;
    }
    return distance;
  }

  public double getBestDistanceRedo() {
    double distance;
    if (mCameraResult.hasTargets()) {
      distance =
          Math.tan(
                  Units.degreesToRadians(
                      mCameraResult.getBestTarget().getPitch()
                          + Units.radiansToDegrees(VisionConstants.kCameraMountPitchAngle)))
              / (VisionConstants.kTargetHeightMeters - VisionConstants.kCameraHeightMeters);
    } else {
      distance = 0;
    }
    return distance;
  }

  public boolean turnShooterToTarget() {
    boolean turnedOnTarget = false;
    double rotationSpeed;

    // if (xboxController.getRightBumper()) { // switch to joystick button
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    if (hasTargets()) {
      rotationSpeed = -turnController.calculate(getTargetYaw(), 0);
      drivetrain.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
      turnedOnTarget = turnController.atSetpoint();
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
    // drivetrain.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
    // }

    return turnedOnTarget;
  }

  public double TurnShooterToTargetOutput() {
    double rotationSpeed = 0;
    turnController.setTolerance(0);
    if (hasTargets()) {
      rotationSpeed =
          -turnController.calculate(getTargetYaw7(), 0) /*/ + turnfeed.calculate(1, 0.25)*/;
    }
    return rotationSpeed;
  }

  public double GetDistance() {
    // var result = mVisionCamera.getLatestResult();
    double range;

    if (mCameraResult.hasTargets()) {
      range =
          PhotonUtils.calculateDistanceToTargetMeters(
                  VisionConstants.kCameraHeightMeters,
                  VisionConstants.kTargetHeightMeters,
                  VisionConstants.kCameraMountPitchAngle,
                  Units.degreesToRadians(-mCameraResult.getBestTarget().getPitch()))
              * 2;
    } else {
      range = 0;
    }
    return range;
  }

  public double pivotShooterToTargetOutput() {
    // pivotController.setTolerance(0);

    if (hasTargets() && GetDistance() != 0) {
      double distance = getTargetDistance() /* - Units.inchesToMeters(12)*/;
      double targetHeightMeters = Units.inchesToMeters(78 + 7 - 11.5);
      pivotAngle = (Math.atan(targetHeightMeters / distance) * 180) / Math.PI;
      // pivotAngle = -pivotController.calculate(Measurement, angleToTarget);
    } else {
      if (pivotAngle == 100000) {
        pivotAngle = 40;
      }
    }
    return pivotAngle + 11;
    // return MathUtil.clamp(pivotAngle , 40, 62);//+ 4.5 + 2 + 0.5
  }

  // Use our forward/turn speeds to control the drivetrain
  // drive.arcadeDrive(forwardSpeed, rotationSpeed);
  // return turnedOnTarget

  // Use our forward/turn speeds to control the drivetrain
  // drive.arcadeDrive(forwardSpeed,rotationSpeed);

  public double getPoseAmbiguity() {

    return getBestTarget().getPoseAmbiguity();
  }

  // Get the transform that maps camera space (X = forward, Y = left, Z = up) to
  // object/fiducial tag space (X forward, Y left, Z up)
  private void updateSmartDashboardGyro() {
    if (Constants.IMUConstants.kGyroDeviceType == "navX") {
      if (ahrs.isConnected()) {
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());
        // /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect)
        // */
        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

        // /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        // /* NOTE: These values are not normally necessary, but are made available */
        // /* for advanced users. Before using this data, please consider whether */
        // /* the processed data (see above) will suit your needs. */

        SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
        // /* Display estimates of velocity/displacement. Note that these values are */
        // /* not expected to be accurate enough for estimating robot position on a */
        // /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding
        // */
        // /* of these errors due to single (velocity) integration and especially */
        // /* double (displacement) integration. */

        SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
        SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

        // SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

        // /* Omnimount Yaw Axis Information */
        // /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

        // /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

        // boolean zero_yaw_pressed = joystick.getTrigger();
        // if (zero_yaw_pressed) {
        // ahrs.zeroYaw();
        // }
      }
      ;
    }

    SmartDashboard.putNumber("IMU_Yaw", getYaw());
    // SmartDashboard.putNumber("IMU_Pitch", getPitch());
  }

  // Update the smart dashboard
  private void updateSmartDashboard() {
    // Put targets? value
    SmartDashboard.putBoolean("Targets?", hasTargets());

    // Check if targets are found before putting values to prevent null!
    if (hasTargets()) {

      SmartDashboard.putString("Target ID", getTargetID() + "");
      SmartDashboard.putString("Target Area", getTargetArea() + "%");
      SmartDashboard.putString("Target Pitch", getTargetPitch() + "");
      SmartDashboard.putString("Target Yaw", getTargetYaw() + "");
      SmartDashboard.putString("Target Height", getTargetTransformHeight() + "");
      SmartDashboard.putNumber("Camera Height", Constants.VisionConstants.kCameraHeightMeters);
      SmartDashboard.putNumber("Camera Pitch", Constants.VisionConstants.kCameraMountPitchAngle);
      SmartDashboard.putString("Camera Name", Constants.VisionConstants.cameraName);
      SmartDashboard.putNumber(
          "Target Distance X-Plane",
          Units.metersToInches(getTargetDistance())); // OK//m_targetDistance

    } else {
      SmartDashboard.putString("Target ID", "No ID Found!");
      SmartDashboard.putString("Target Pitch", "-1");
      SmartDashboard.putString("Target Area", "0" + "%");
      SmartDashboard.putNumber("Target Distance X-Plane", -1);
    }

    SmartDashboard.putString("LED State", mVisionCamera.getLEDMode().toString());
    // SmartDashboard.putBoolean("m_FieldToRobotAprilTagPoseNull",
    // m_FieldToRobotAprilTagPoseNull);
    // SmartDashboard.putBoolean("m_RobotPose3dNull", m_RobotPose3dNull);
    // SmartDashboard.putBoolean("m_fieldRobotPoseNull", m_fieldRobotPoseNull);

  }

  public void setLEDOn() {
    mVisionCamera.setLED(VisionLEDMode.kBlink);
    DriverStation.reportWarning("CHANGE LED", true);
  }

  // A periodic loop, updates smartdashboard and camera results
  @Override
  public void periodic() {
    // drivetrain.applyRequest(() -> drive.withRotationalRate(-100));

    // SmartDashboard.putNumber("getTranslationDistnace", GetTranslationDistance());

    SmartDashboard.putNumber("GetYaw7", getTargetYaw7());

    // SmartDashboard.putBoolean("Has7", mCameraResult.getMultiTagResult().);

    SmartDashboard.putNumber("GetDistance", Units.metersToInches(GetDistance()));
    SmartDashboard.putNumber("GetBestDistance", Units.metersToInches(getBestDistanceRedo()));

    SmartDashboard.putBoolean("hastargetsstss", VisionSubsystem.HasTargets);
    // SmartDashboard.putNumber("Skew", mCameraResult.hasTargets() ?
    // 0:mVisionCamera.getLatestResult().getBestTarget().getSkew());

    SmartDashboard.putNumber("GetDistanceFranco", Units.metersToInches(getDistance()));
    SmartDashboard.putNumber("ChassisOT", Units.degreesToRadians(TurnShooterToTargetOutput()));
    SmartDashboard.putString("Camera", mVisionCamera.toString());
    updateCameraResults();
    updatePoses();
    updateSmartDashboard();
    // updateSmartDashboardGyro();
    // turnShooterToTarget();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void close() {
    pigeon.close();
  }
}
