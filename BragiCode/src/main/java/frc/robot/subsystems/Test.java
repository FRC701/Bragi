package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {
  private AHRS navX;
  private int device;
  private SimDouble angle;
  public XboxController XboxController = new XboxController(0);

  public Test() {
    SmartDashboard.putBoolean("Reset Yaw", false);
    navX = new AHRS(SPI.Port.kMXP);
    device = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, "Yaw"));
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public void resetYaw() {
    navX.zeroYaw();
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public double getPitch() {
    return navX.getPitch();
  }

  public boolean isConnected() {
    return navX.isConnected();
  }

  public boolean isCalibrating() {
    return navX.isCalibrating();
  }

  public double getCompassHeading() {
    return navX.getCompassHeading();
  }

  public double getFusedHeading() {
    return navX.getFusedHeading();
  }

  public double getVelocityX() {
    return navX.getVelocityX();
  }

  public double getVelocityY() {
    return navX.getVelocityY();
  }

  public double getVelocityZ() {
    return navX.getVelocityZ();
  }

  public double getDisplacementY() {
    return navX.getDisplacementY();
  }

  public double getDisplacementX() {
    return navX.getDisplacementX();
  }

  public double getRate() {
    return navX.getRate();
  }

  public double getWorldLinearAccelX() {
    return navX.getWorldLinearAccelX();
  }

  public double getWorldLinearAccelY() {
    return navX.getWorldLinearAccelY();
  }

  public double getRawGyroX() {
    return navX.getRawGyroX();
  }

  public double getRawGyroY() {
    return navX.getRawGyroY();
  }

  public double getRawGyroZ() {
    return navX.getRawGyroZ();
  }

  public double getRawAccelX() {
    return navX.getRawAccelX();
  }

  public double getRawAccelY() {
    return navX.getRawAccelY();
  }

  public double getRawAccelZ() {
    return navX.getRawAccelZ();
  }

  public double getRawMagX() {
    return navX.getRawMagX();
  }

  public double getRawMagY() {
    return navX.getRawMagY();
  }

  public double getRawMagZ() {
    return navX.getRawMagZ();
  }

  public double getQuaternionW() {
    return navX.getQuaternionW();
  }

  public double getQuaternionX() {
    return navX.getQuaternionX();
  }

  public double getQuaternionZ() {
    return navX.getQuaternionZ();
  }

  public double getQuaternionY() {
    return navX.getQuaternionY();
  }

  public double getTempC() {
    return navX.getTempC();
  }

  public double getLastSensorTimestamp() {
    return navX.getLastSensorTimestamp();
  }

  public double getByteCount() {
    return navX.getByteCount();
  }

  public double getUpdateCount() {
    return navX.getUpdateCount();
  }

  public String getFirmwareVersion() {
    return navX.getFirmwareVersion();
  }

  public boolean isMoving() {
    return navX.isMoving();
  }

  public boolean isRotating() {
    return navX.isRotating();
  }

  public AHRS.BoardYawAxis getBoardYawAxis() {
    return navX.getBoardYawAxis();
  }

  /* Display Raw Gyro/Accelerometer/Magnetometer Values */
  /* NOTE: These values are not normally necessary, but are made available */
  /* for advanced users. Before using this data, please consider whether */
  /* the processed data (see above) will suit your needs. */

  @Override
  public void periodic() {

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", getYaw());
    SmartDashboard.putNumber("IMU_Pitch", getPitch());
    SmartDashboard.putNumber("IMU_Roll", getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", isRotating());
    SmartDashboard.putBoolean("button A is pressed", XboxController.getAButtonPressed());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", getVelocityY());
    SmartDashboard.putNumber("Displacement_X", getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", getUpdateCount());

    SmartDashboard.putNumber("Yaw:", getYaw());
    SmartDashboard.putNumber("Angle:", getAngle());
    SmartDashboard.putNumber("SimAngle:", angle.get());
    if (SmartDashboard.getBoolean("Reset Yaw", false)) resetYaw();
    boolean zero_yaw_pressed = XboxController.getAButtonPressed();
    if (zero_yaw_pressed) {
      resetYaw();
    }
  }
}
