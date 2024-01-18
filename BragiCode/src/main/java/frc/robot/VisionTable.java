// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.temporal.ValueRange;

/** Add your docs here. */

public class VisionTable {
    public int distance;
    public double targetArea;
    public double Velocity;
   /*  public int[][] DataTable =  {{0, 1, 2},
                                 {0, 1, 2},
                                 {0, 1, 2}};*/

    VisionTable DataTable[] = new VisionTable[]{
        new VisionTable(1, 2, 3),
        new VisionTable(0, 0, 0),
        new VisionTable(0, 0, 0)
    };

    private ValueRange range;
    private boolean inRange;

public String GetBound(double TA)
{
    int iTa = Double.valueOf(TA).intValue();
    for(int i = 0 ; inRange ; i++)
    {
        range = java.time.temporal.ValueRange.of(DataTable[i].distance, DataTable[i + 1].distance); //DataTable[i][0], DataTable[i+1][]
        inRange = range.isValidIntValue(iTa);
    }
    return "currentRange =" + range.getMinimum() + "-" + range.getMaximum();
}

public VisionTable(int distance, double targetArea, double Velocity)
{
    this.distance = distance;
    this.targetArea = targetArea;
    this.Velocity = Velocity;
}
}

