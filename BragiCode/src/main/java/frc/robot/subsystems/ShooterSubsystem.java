// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.FeederEnumState;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX mShooterMotorTop;
  private TalonFX mShooterMotorBottom;

  public static double mSmartSpeed = 0;

  public static double InputVelocity;

  public static ShooterState mShooterState;

  // private Timer mTimer;

  // private Feeder mFeeder;

  private int counter = 0;

  private boolean HasPassedSetpoint = false;
  private boolean SetpointMet = false;

  private boolean Ready = false;

  public static Boolean AutoAim = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = Constants.ShooterConstants.kV;
    Slot0Configs.kP = Constants.ShooterConstants.kP;
    Slot0Configs.kI = Constants.ShooterConstants.kI;
    Slot0Configs.kD = Constants.ShooterConstants.kD;

    var Slot1Configs = new Slot1Configs();
    Slot1Configs.kV = 0.5;

    mShooterMotorTop = new TalonFX(Constants.ShooterConstants.kShooterMotorTop, "Cani");
    mShooterMotorBottom = new TalonFX(Constants.ShooterConstants.kShooterMotorBottom, "Cani");

    // mTimer = new Timer();

    // mFeeder = new Feeder();

    mShooterMotorTop.getConfigurator().apply(Slot0Configs, 0.05);
    mShooterMotorTop.getConfigurator().apply(Slot1Configs, 0.05);

    mShooterMotorBottom.setControl(
        new Follower(mShooterMotorTop.getDeviceID(), Constants.kOpposeMasterDirection));
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
    VelocityVoltage VeloSpeed = new VelocityVoltage(-0.5).withSlot(1);
    mShooterMotorTop.setControl(VeloSpeed);
  }

  public void AccelerateShooter() {
    if (Ready) {
      mShooterState = ShooterState.S_Shoot;
      Feeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
    } else {
      VelocityVoltage VeloSpeed = new VelocityVoltage(mSmartSpeed).withSlot(0);
      mShooterMotorTop.setControl(VeloSpeed);
      CheckShooterUpToSpeed();
    }
  }

  public void Shoot() {
    VelocityVoltage VeloSpeed = new VelocityVoltage(mSmartSpeed).withSlot(0);
    mShooterMotorTop.setControl(VeloSpeed);
    Ready = false;
    counter = 0;
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

    return ShooterVelo(mShooterMotorTop) < max && ShooterVelo(mShooterMotorTop) > min;
  }

  private double ShooterVelo(TalonFX motorFx) {
    return motorFx.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("AutoAim", AutoAim);
    SmartDashboard.putNumber("ShooterSpeed", -ShooterVelo(mShooterMotorTop));

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
