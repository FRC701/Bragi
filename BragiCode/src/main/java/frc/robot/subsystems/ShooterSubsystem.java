// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.FeederEnumState;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX mShooterMotorLeft;
  private TalonFX mShooterMotorRight;

  public static double mSmartSpeed = 0;

  public static double InputVelocity;

  public static ShooterState mShooterState;

  private Timer mTimer;

  private int counter = 0;

  private boolean HasPassedSetpoint = false;
  private boolean SetpointMet = false;

  private boolean Ready = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = 0.135;
    Slot0Configs.kP = 0.;
    Slot0Configs.kI = 0;
    Slot0Configs.kD = 0.00;

    mShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    mShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

    mTimer = new Timer();

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
    mShooterMotorLeft.set(0);
  }

  public void AccelerateShooter() {
    if (Ready) {
      mShooterState = ShooterState.S_Shoot;
      Feeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
      Ready = false;
      counter = 0;
    } else {
      VelocityVoltage VeloSpeed = new VelocityVoltage(mSmartSpeed);
      mShooterMotorLeft.setControl(VeloSpeed);
      CheckShooterUpToSpeed();
    }
  }

  public void Shoot() {
    if (mTimer.hasElapsed(1.5)) {
      mTimer.stop();
      mTimer.reset();
      mShooterState = ShooterState.S_WaitingForFeeder;
      Feeder.mFeederEnumState = FeederEnumState.S_WaitingOnNote;
    } else {
      VelocityVoltage VeloSpeed = new VelocityVoltage(mSmartSpeed);
      mShooterMotorLeft.setControl(VeloSpeed);
      mTimer.start();
    }
  }

  public void CheckShooterUpToSpeed() {

    if (WithinHistorises()) {
      SetpointMet = true;
    } else if (SetpointMet && !WithinHistorises()) {
      HasPassedSetpoint = true;
    }

    if (HasPassedSetpoint && SetpointMet) {
      counter = counter + 1;
      SetpointMet = false;
      HasPassedSetpoint = false;
    }
    if (counter >= 2) {
      Ready = true;
    }
  }

  public boolean WithinHistorises() {
    double max = mSmartSpeed - 0.003 * mSmartSpeed;
    double min = mSmartSpeed + 0.003 * mSmartSpeed;
    SmartDashboard.putNumber("min", min);
    SmartDashboard.putNumber("max", max);

    return ShooterVelo(mShooterMotorLeft) < max && ShooterVelo(mShooterMotorLeft) > min;
  }

  private double ShooterVelo(TalonFX motorFx) {
    return motorFx.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", -ShooterVelo(mShooterMotorLeft));
    SmartDashboard.putString("ShooterState", mShooterState.toString());
    SmartDashboard.putNumber("Counter", counter);
    SmartDashboard.putBoolean("IsReady?", Ready);
    SmartDashboard.putBoolean("HasPassedSetpoint", HasPassedSetpoint);
    SmartDashboard.putBoolean("SetpointMet", SetpointMet);
    SmartDashboard.putNumber("SmartSpeed", -mSmartSpeed);
    SmartDashboard.putBoolean("WithinHist", WithinHistorises());

    InputVelocity = -SmartDashboard.getNumber("Input Velocity", 0);

    RunShooterState();

    // This method will be called once per scheduler run
  }
}
