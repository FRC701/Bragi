package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class FieldLayout {

  /** Origin is the bottom left corner of the field image (Blue Alliance Station) */
  public static final double kFieldLength = Units.inchesToMeters(653.2);

  public static final double kFieldWidth = Units.inchesToMeters(293.64);

  // AprilTag locations
  public static final Map<Integer, Pose3d> aprilTags =
      Map.ofEntries(
          Map.entry(
              1, // Blue Source (right)
              new Pose3d(
                  Units.inchesToMeters(593.68),
                  Units.inchesToMeters(9.68),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(120)))),
          Map.entry(
              2, // Blue Source (left)
              new Pose3d(
                  Units.inchesToMeters(637.21),
                  Units.inchesToMeters(34.79),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(120)))),
          Map.entry(
              3, // Red Speaker (right)
              new Pose3d(
                  Units.inchesToMeters(652.73),
                  Units.inchesToMeters(196.17),
                  Units.inchesToMeters(57.13),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)))),
          Map.entry(
              4, // Red Speaker(right)
              new Pose3d(
                  Units.inchesToMeters(652.73),
                  Units.inchesToMeters(218.42),
                  Units.inchesToMeters(57.13),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)))),
          Map.entry(
              5, // Red Amp
              new Pose3d(
                  Units.inchesToMeters(578.77),
                  Units.inchesToMeters(323),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0, 0, Units.degreesToRadians(270)))),
          Map.entry(
              6, // Blue Amp
              new Pose3d(
                  Units.inchesToMeters(72.5),
                  Units.inchesToMeters(323),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0, 0, Units.degreesToRadians(270)))),
          Map.entry(
              7, // Blue Speaker (right)
              new Pose3d(
                  Units.inchesToMeters(-1.5),
                  Units.inchesToMeters(218.32),
                  Units.inchesToMeters(57.13),
                  new Rotation3d(0, 0, Units.degreesToRadians(0)))),
          Map.entry(
              8, // Blue Speaker (left)
              new Pose3d(
                  Units.inchesToMeters(-1.5),
                  Units.inchesToMeters(196.17),
                  Units.inchesToMeters(57.13),
                  new Rotation3d(0, 0, Units.degreesToRadians(0)))),
          Map.entry(
              9, // Red Source (right)
              new Pose3d(
                  Units.inchesToMeters(14.02),
                  Units.inchesToMeters(34.79),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0, 0, Units.degreesToRadians(60)))),
          Map.entry(
              10, // Red Source (left)
              new Pose3d(
                  Units.inchesToMeters(57.54),
                  Units.inchesToMeters(9.68),
                  Units.inchesToMeters(53.38),
                  new Rotation3d(0, 0, Units.degreesToRadians(60)))),
          Map.entry(
              11, // Red Stage (Stage Left)
              new Pose3d(
                  Units.inchesToMeters(468.69),
                  Units.inchesToMeters(146.19),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(300)))),
          Map.entry(
              12, // Red Stage (Center Stage)
              new Pose3d(
                  Units.inchesToMeters(468.69),
                  Units.inchesToMeters(177.1),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(60)))),
          Map.entry(
              13, // Red Stage (Stage Light)
              new Pose3d(
                  Units.inchesToMeters(441.74),
                  Units.inchesToMeters(161.62),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(180)))),
          Map.entry(
              14, // Blue Stage (Center Stage)
              new Pose3d(
                  Units.inchesToMeters(209.48),
                  Units.inchesToMeters(161.62),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(0)))),
          Map.entry(
              15, // Blue Stage (Stage Right)
              new Pose3d(
                  Units.inchesToMeters(182.73),
                  Units.inchesToMeters(177.1),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(120)))),
          Map.entry(
              16, // Blue Stage (Stage Left)
              new Pose3d(
                  Units.inchesToMeters(182.73),
                  Units.inchesToMeters(146.19),
                  Units.inchesToMeters(52),
                  new Rotation3d(0, 0, Units.degreesToRadians(240)))));
}
