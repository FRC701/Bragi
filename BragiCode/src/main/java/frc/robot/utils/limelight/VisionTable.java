// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.limelight;

import java.time.temporal.ValueRange;

/** Add your docs here. */
public class VisionTable {
  public static final double CameraHeight = 123456789;
  public static final double SpeakerHeight = 987654321;

  public int[][] DataTable = {
    {1, 0},
    {5, 50},
    {10, 100}
  };

  // {distance, rpm)

  public double GetSpeed(double Distance) {
    double Slope =
        (GetBound(Distance)[1][1] - GetBound(Distance)[0][1])
            / (GetBound(Distance)[1][0] - GetBound(Distance)[0][0]);
    double Speed = Slope * (Distance - GetBound(Distance)[0][0]) + GetBound(Distance)[0][1];
    return Speed;
  }

  public double GetAngle(double Distance) {
    double Angle = Math.tan((SpeakerHeight - CameraHeight) / Distance);
    return Angle;
  }

  public double[][] GetBound(double Distance) {
    int iTa = (int) Distance;
    ValueRange range;
    int i = 0;
    if (iTa > DataTable[DataTable.length][0] || iTa < DataTable[0][0]) {
      double[][] outlier = {{0, 0}, {0, 0}};
      return outlier;
    } else {
      for (; i < DataTable.length; i++) {
        range = java.time.temporal.ValueRange.of(DataTable[i][0], DataTable[i + 1][0]);
        if (range.isValidIntValue(iTa)) {
          break;
        }
      }
      double[][] bound = {
        {DataTable[i][0], DataTable[i][1]}, {DataTable[i + 1][0], DataTable[i + 1][1]}
      };
      return bound;
    }
  }
}
