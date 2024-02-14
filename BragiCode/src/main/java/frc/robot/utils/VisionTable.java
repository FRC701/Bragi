// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.time.temporal.ValueRange;

/** Add your docs here. */
public class VisionTable {
  public int[][] DataTable = {
    {1, 0, 2},
    {5, 50, 2},
    {10, 100, 2}
  };

  // {target, rpm, distance}

  public double[] GetBound(double TA) {
    int iTa = (int) TA;
    ValueRange range;
    int i = 0;
    if (iTa > DataTable[DataTable.length][0] || iTa < DataTable[0][0]) {
      double[] outlier = {0, 0};
      return outlier;
    } else {
      for (; i < DataTable.length; i++) {
        range = java.time.temporal.ValueRange.of(DataTable[i][0], DataTable[i + 1][0]);
        if (range.isValidIntValue(iTa)) {
          break;
        }
      }
      double[] bound = {DataTable[i][1], DataTable[i + 1][1]};
      return bound;
    }
  }
}
