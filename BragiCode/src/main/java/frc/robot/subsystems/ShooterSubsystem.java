// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.FeederEnumState;

public class ShooterSubsystem extends SubsystemBase {
  public TalonFX mShooterMotorLeft;
  public TalonFX mShooterMotorRight;

  private Feeder mFeeder;

  public double mSmartSpeed = 0;

  public VelocityDutyCycle VeloSpeed = new VelocityDutyCycle(mSmartSpeed);

  public ShooterState mShooterState;

  private Timer mTimer = new Timer();

  public enum ShooterState {
    S_Shoot,
    S_WaitingForFeeder,
    S_AccelerateShooter
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    mShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    mShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

    mShooterMotorRight.setControl(
        new Follower(mShooterMotorLeft.getDeviceID(), Constants.kOpposeMasterDirection));
    mShooterState = ShooterState.S_WaitingForFeeder;
  }

  public void RunShooterState() {
    switch (mShooterState) {
      case S_WaitingForFeeder:
        WaitingForFeeder();
        break;
      case S_AccelerateShooter:
        AccelerateShooter();
        break;
      case S_Shoot:
        Shoot();
        break;
    }
  }

  public void WaitingForFeeder() {
    mShooterMotorLeft.set(0.1);
  }

  public void AccelerateShooter() {
    mShooterMotorLeft.setControl(VeloSpeed);
    if (IsShooterUpToSpeed()) {
      mShooterState = ShooterState.S_Shoot;
      mFeeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
    }
  }

  public void Shoot() {
    mShooterMotorLeft.setControl(VeloSpeed);
    mTimer.start();
    if (mTimer.hasElapsed(3)) {
      mTimer.stop();
      mTimer.reset();
      mShooterState = ShooterState.S_WaitingForFeeder;
      mFeeder.mFeederEnumState = FeederEnumState.S_WaitingOnNote;
    }
  }

  public boolean IsShooterUpToSpeed() {
    boolean CounterPassed = false;
    boolean HasPassedSetpoint = false;
    boolean SetpointMet = false;
    double min = mSmartSpeed - 0.1 * mSmartSpeed;
    double max = mSmartSpeed + 0.1 * mSmartSpeed;
    for (int x = 0; x < 5; x++) {
      while (!CounterPassed) {
        if (ShooterVelo(mShooterMotorLeft) < max && ShooterVelo(mShooterMotorLeft) > min) {
          SetpointMet = true;
        } else if (SetpointMet && ShooterVelo(mShooterMotorLeft) != mSmartSpeed) {
          HasPassedSetpoint = true;
        } else if (HasPassedSetpoint && SetpointMet) {
          CounterPassed = true;
        }
      }
      CounterPassed = false;
      SetpointMet = false;
      HasPassedSetpoint = false;
    }
    return true;
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
