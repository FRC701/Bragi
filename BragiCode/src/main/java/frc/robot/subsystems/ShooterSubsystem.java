// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public TalonFX mShooterMotorLeft;
  public TalonFX mShooterMotorRight;

  public ShooterState mShooterState;

  private Timer mTimer = new Timer();

  public enum ShooterState {
    S_Shoot,
    S_WaitingForFeeder
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    mShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

    mShooterMotorRight.setControl(
        new Follower(mShooterMotorLeft.getDeviceID(), Constants.kOpposeMasterDirection));
  }

  public void RunShooterState() {
    switch (mShooterState) {
      case S_WaitingForFeeder:
        WaitingForFeeder();
        break;
      case S_Shoot:
        Shoot();
        break;
    }
  }

  public void WaitingForFeeder() {
    mShooterMotorLeft.set(0.5);
  }

  public void Shoot() {
    mTimer.start();
    if (mTimer.hasElapsed(5)) {
      mTimer.stop();
      mTimer.reset();
      mShooterState = ShooterState.S_WaitingForFeeder;
    }
  }

  private static double ShooterVelo(TalonFX motorFx) {
    return motorFx.getVelocity().getValueAsDouble();
  }

  public void Velocity() {
    SmartDashboard.putNumber("ShooterMotorRight", ShooterVelo(mShooterMotorRight));
    SmartDashboard.putNumber("ShooterMotorLeft", ShooterVelo(mShooterMotorLeft));
  }

  @Override
  public void periodic() {
    RunShooterState();
    Velocity();
    // This method will be called once per scheduler run
  }
}
