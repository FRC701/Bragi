// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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

  public ShooterState mShooterState;

  private Timer mTimer = new Timer();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = 0.0115;
    Slot0Configs.kP = 0.03;
    Slot0Configs.kI = 0;
    Slot0Configs.kD = 0.001;

    mShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    mShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

    mShooterMotorLeft.getConfigurator().apply(Slot0Configs, 0.05);

    mShooterMotorRight.setControl(
        new Follower(mShooterMotorLeft.getDeviceID(), Constants.kOpposeMasterDirection));
    mShooterState = ShooterState.S_WaitingForFeeder;
  }

  public enum ShooterState {
    S_Shoot,
    S_WaitingForFeeder,
    S_AccelerateShooter
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
    VelocityDutyCycle VeloSpeed = new VelocityDutyCycle(mSmartSpeed);
    mShooterMotorLeft.setControl(VeloSpeed);
    if (IsShooterUpToSpeed()) {
      mShooterState = ShooterState.S_Shoot;
      mFeeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
    }
  }

  public void Shoot() {
    VelocityDutyCycle VeloSpeed = new VelocityDutyCycle(mSmartSpeed);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", ShooterVelo(mShooterMotorLeft));
    SmartDashboard.putBoolean("UpToSpeed", IsShooterUpToSpeed());
    SmartDashboard.putString("ShooterState", "=" + mShooterState);
    RunShooterState();
    // This method will be called once per scheduler run
  }
}
