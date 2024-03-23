// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
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
  public static double mShooterMotorBottom_current;
  public static double mShooterMotorTop_current;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = Constants.ShooterConstants.kVt;
    Slot0Configs.kP = Constants.ShooterConstants.kPt;
    Slot0Configs.kI = Constants.ShooterConstants.kIt;
    Slot0Configs.kD = Constants.ShooterConstants.kDt;
    Slot0Configs.kA = Constants.ShooterConstants.kAt; // 50
    Slot0Configs.kS = 0.40;

    var Slot1Configs = new Slot1Configs();
    Slot1Configs.kV = 1;

    var Slot0Configs0 = new Slot0Configs();
    Slot0Configs0.kV = Constants.ShooterConstants.kVb;
    Slot0Configs0.kP = Constants.ShooterConstants.kPb;
    Slot0Configs0.kI = Constants.ShooterConstants.kIb;
    Slot0Configs0.kD = Constants.ShooterConstants.kDb;
    Slot0Configs0.kA = Constants.ShooterConstants.kAb;
    Slot0Configs0.kS = 0.45;

    var Slot1Configs0 = new Slot1Configs();
    Slot1Configs0.kV = 1.5;

    mShooterMotorTop =
        new TalonFX(Constants.ShooterConstants.kShooterMotorTop, Constants.kCanivoreBusName);
    mShooterMotorBottom =
        new TalonFX(Constants.ShooterConstants.kShooterMotorBottom, Constants.kCanivoreBusName);

    // mTimer = new Timer();

    // mFeeder = new Feeder();

    mShooterMotorTop.getConfigurator().apply(Slot0Configs, 0.05);
    // mShooterMotorTop.getConfigurator().apply(Slot1Configs, 0.05);

    mShooterMotorBottom.getConfigurator().apply(Slot0Configs0, 0.05);
    // mShooterMotorBottom.getConfigurator().apply(Slot1Configs0, 0.05);

    // mShooterMotorBottom.setControl(new Follower(mShooterMotorTop.getDeviceID(), false));

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
    Ready = false;
    counter = 0;
    VelocityVoltage VeloSpeed = new VelocityVoltage(0.75).withSlot(1);
    mShooterMotorTop.setControl(VeloSpeed);
    mShooterMotorBottom.setControl(VeloSpeed);
  }

  public void AccelerateShooter() {
    if (ShooterVelo(mShooterMotorBottom)
        >= mSmartSpeed - 10) { // ShooterVelo(mShooterMotorBottom) >= mSmartSpeed - 0.75
      mShooterState = ShooterState.S_Shoot;
      Feeder.mFeederEnumState = FeederEnumState.S_ShooterReady;
    } else {
      // VelocityVoltage VeloSpeed = new VelocityVoltage(mSmartSpeed).withSlot(0);

      VelocityVoltage TopSpeed =
          new VelocityVoltage(mSmartSpeed * ShooterConstants.kShooterTopReduction).withSlot(0);
      // VelocityVoltage BottomSpeed =
      //     new VelocityVoltage(mSmartSpeed * ShooterConstants.kShooterBottomReduction)
      //         .withSlot(
      //             0); // KYLE THIS CODE MAKES IT SO THAT THE PID's SETPOINT VELOCITY IS THE
      // SETPOINT
      // VELOCITY OF THE FINAL OUTPUT SHAFT
      // WHEN OBSERVING SPEEDS REMEBER YOUR MOTOR SETPOINT WONT NECASSARILY BE YOUR INPUTVELOCITY;

      mShooterMotorTop.setControl(TopSpeed);
      mShooterMotorBottom.setControl(TopSpeed);
      CheckShooterUpToSpeed();
    }
  }

  public void Shoot() {
    VelocityVoltage TopSpeed =
        new VelocityVoltage(mSmartSpeed * ShooterConstants.kShooterTopReduction).withSlot(0);
    // VelocityVoltage BottomSpeed =
    //     new VelocityVoltage(mSmartSpeed * ShooterConstants.kShooterBottomReduction).withSlot(0);
    mShooterMotorTop.setControl(TopSpeed);
    mShooterMotorBottom.setControl(TopSpeed);
    // mShooterMotorBottom.setControl(BottomSpeed);
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
    if (counter >= 1) {
      Ready = true;
    }
  }

  public boolean WithinHistorises() {
    double max = mSmartSpeed - 0.0001 * mSmartSpeed;
    double min = mSmartSpeed + 0.0001 * mSmartSpeed;
    SmartDashboard.putNumber("min", min);
    SmartDashboard.putNumber("max", max);

    return ShooterVelo(mShooterMotorBottom) > max && ShooterVelo(mShooterMotorBottom) < min;
  }

  private double ShooterVelo(TalonFX motorFx) {
    return motorFx.getVelocity().getValueAsDouble();
  }

  private double TopFinalVelo() {
    return mShooterMotorTop.getVelocity().getValueAsDouble()
        * ShooterConstants.kShooterTopReduction;
  }

  private double BottomFinalVelo() {
    return mShooterMotorBottom.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("AutoAim", AutoAim);
    SmartDashboard.putNumber("ShooterSpeedTop", -ShooterVelo(mShooterMotorTop));
    SmartDashboard.putNumber("ShooterSpeedBottom", -ShooterVelo(mShooterMotorBottom));
    SmartDashboard.putNumber("FinalShooterSpeedTop", -TopFinalVelo());
    SmartDashboard.putNumber("FinalShooterSpeedBottom", -BottomFinalVelo());

    SmartDashboard.putString("ShooterState", mShooterState.toString());
    SmartDashboard.putNumber("Counter", counter);
    SmartDashboard.putBoolean("IsReady?", Ready);
    SmartDashboard.putBoolean("HasPassedSetpoint", HasPassedSetpoint);
    SmartDashboard.putBoolean("SetpointMet", SetpointMet);
    SmartDashboard.putNumber("SmartSpeed", -mSmartSpeed);
    SmartDashboard.putBoolean("WithinHist", WithinHistorises());

    InputVelocity = -SmartDashboard.getNumber("Input Velocity", 0);
    mShooterMotorTop_current = mShooterMotorTop.getSupplyCurrent().getValue();
    SmartDashboard.putNumber("mShooterMotorTop_current", mShooterMotorTop_current);
    mShooterMotorBottom_current = mShooterMotorBottom.getSupplyCurrent().getValue();
    SmartDashboard.putNumber("mShooterMotorBottom_current", mShooterMotorBottom_current);
    RunShooterState();

    // This method will be called once per scheduler run
  }
}
