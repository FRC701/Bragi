package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class kVision {
  public static final String DRIVER_CAM_URL = "http://photonvision.local:5800/";

  public static final double CAMERA_HEIGHT = Units.inchesToMeters(28);
  public static final double TARGET_HEIGHT = Units.inchesToMeters(104);
  public static final double CAMERA_PITCH = Units.degreesToRadians(50);
  public static final double FIELD_WIDTH = 16.5; // m
  public static final double FIELD_HEIGHT = 8.1; // m

  public static final double DEBOUNCE_TIME = 0.1;
}
