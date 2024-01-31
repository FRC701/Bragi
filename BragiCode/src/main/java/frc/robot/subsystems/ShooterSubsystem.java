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
  private TalonFX mShooterMotorLeft;
  private TalonFX mShooterMotorRight;

  public double mSmartSpeed = 0;

  public static ShooterState mShooterState;

  private Timer mTimer = new Timer();

  private int counter = 0;

  private boolean Ready;

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

    mShooterMotorRight.setControl(new Follower(mShooterMotorLeft.getDeviceID(), true));
    mShooterState = ShooterState.S_WaitingForFeeder;
  }

  public enum ShooterState {
    S_WaitingForFeeder,
    S_AccelerateShooter,
    S_Shoot
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
    if (Ready) {
      mShooterState = ShooterState.S_Shoot;
      Feeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
      Ready = false;
      counter = 0;
    } else {
      VelocityDutyCycle VeloSpeed = new VelocityDutyCycle(mSmartSpeed);
      mShooterMotorLeft.setControl(VeloSpeed);
      CheckShooterUpToSpeed();
    }
  }

  public void Shoot() {
    if (mTimer.hasElapsed(3)) {
      mTimer.stop();
      mTimer.reset();
      mShooterState = ShooterState.S_WaitingForFeeder;
      Feeder.mFeederEnumState = FeederEnumState.S_WaitingOnNote;
    } else {
      VelocityDutyCycle VeloSpeed = new VelocityDutyCycle(mSmartSpeed);
      mShooterMotorLeft.setControl(VeloSpeed);
      mTimer.start();
    }
  }

  public void CheckShooterUpToSpeed() {
    boolean HasPassedSetpoint = false;
    boolean SetpointMet = false;
    double min = mSmartSpeed - 0.1 * mSmartSpeed;
    double max = mSmartSpeed + 0.1 * mSmartSpeed;
    if (ShooterVelo(mShooterMotorLeft) < max && ShooterVelo(mShooterMotorLeft) > min) {
      SetpointMet = true;
    } else if (SetpointMet && ShooterVelo(mShooterMotorLeft) != mSmartSpeed) {
      HasPassedSetpoint = true;
    } else if (HasPassedSetpoint && SetpointMet) {
      counter++;
      SetpointMet = false;
      HasPassedSetpoint = false;
    } else if (counter >= 5) {
      Ready = true;
    }
  }

  private double ShooterVelo(TalonFX motorFx) {
    return motorFx.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", ShooterVelo(mShooterMotorLeft));
    SmartDashboard.putString("ShooterState", mShooterState.toString());
    SmartDashboard.putNumber("Counter", counter);
    SmartDashboard.putBoolean("IsReady?", Ready);
    RunShooterState();
    // This method will be called once per scheduler run
  }
}
