// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX ElevatorMotorLeft;
  private TalonFX ElevatorMotorRight;

  /** Creates a new Elevator. */
  public Elevator() {
    ElevatorMotorLeft = new TalonFX(Constants.ElevatorConstants.kElevatorMotorLeft);
    ElevatorMotorRight = new TalonFX(Constants.ElevatorConstants.kElevatorMotorRight);
    ElevatorMotorLeft.setControl(
        new Follower(ElevatorMotorRight.getDeviceID(), Constants.kOpposeMasterDirection));
  }

  public void MoveElevator(double speed) {
    ElevatorMotorLeft.setVoltage(speed * 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
