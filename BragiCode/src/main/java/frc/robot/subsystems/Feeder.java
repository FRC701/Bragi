// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeEnumState;
import frc.robot.subsystems.LED.LedState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor;

  public static FeederEnumState mFeederEnumState;

  // private ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

  private TalonFXConfiguration mTalonFXConfig;

  private static Timer Timer;
  public static double kFeederMotor_current;

  public Feeder() {
    FeederMotor = new TalonFX(Constants.FeederConstants.kFeederMotor, "cani");

    mFeederEnumState = FeederEnumState.S_WaitingForIntake;
    Timer = new Timer();
    mTalonFXConfig = new TalonFXConfiguration();
    mTalonFXConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    FeederMotor.getConfigurator().apply(mTalonFXConfig);

    var fx_cfg = new MotorOutputConfigs();

    fx_cfg.NeutralMode = NeutralModeValue.Brake;

    FeederMotor.getConfigurator().apply(fx_cfg);
  }

  public enum FeederEnumState {
    S_WaitingForIntake,
    S_NoteInIntake,
    S_ShooterReady,
    S_funEject;
  }

  public void RunFeederState() {
    switch (mFeederEnumState) {
      case S_WaitingForIntake:
        WaitingForIntake();
        break;
      case S_NoteInIntake:
        NoteInIntake();
        break;
      case S_ShooterReady:
        ShooterReady();
        break;
      case S_funEject:
        funEject();
        break;
    }
  }

  public void WaitingForIntake() {
    if (!revLimitStatus()) {
      Feeder.mFeederEnumState = FeederEnumState.S_NoteInIntake;
      Intake.mIntakeEnumState = IntakeEnumState.S_CarryingNote;
    } else {
      if (Intake.IntakeActive) {
        FeederMotor.setVoltage(-4);
        ;
      } else {
        FeederMotor.setVoltage(0);
        ;
      }
      LED.mLedState = LedState.S_Red;
    }
  }

  public void NoteInIntake() {
    FeederMotor.stopMotor();
    if (ShooterSubsystem.mShooterState == ShooterState.S_AccelerateShooter) {
      LED.mLedState = LedState.S_Pink;
    } else {
      LED.mLedState = LedState.S_Green;
    }
  }

  public void ShooterReady() {
    if (revLimitStatus()) {
      Intake.IntakeActive = false;
      ShooterSubsystem.mShooterState = ShooterState.S_WaitingForFeeder;
      Feeder.mFeederEnumState = FeederEnumState.S_WaitingForIntake;
      Intake.mIntakeEnumState = IntakeEnumState.S_WaitingOnNote;
    } else {
      Intake.mIntakeEnumState = IntakeEnumState.S_IntakeFeed;
      FeederMotor.setVoltage(-8);
      if (ShooterSubsystem.mShooterState == ShooterState.S_Shoot) {
        LED.mLedState = LedState.S_Purple;
      } else {
        LED.mLedState = LedState.S_Blue;
      }
    }

    // Need shoot command and shooter subsystem to be done
    // wait for shooter to become ready
  }

  public void funEject() {
    if (Timer.hasElapsed(0.5)) {
      Timer.stop();
      Timer.reset();
      Intake.IntakeActive = false;
      mFeederEnumState = FeederEnumState.S_WaitingForIntake;
      Intake.mIntakeEnumState = IntakeEnumState.S_WaitingOnNote;
    } else {
      Timer.start();
      FeederMotor.setVoltage(6);
      Intake.mIntakeEnumState = IntakeEnumState.S_Eject;
    }
  }

  public boolean revLimitStatus() {
    return (FeederMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("FeederState", mFeederEnumState.toString());
    RunFeederState();
    SmartDashboard.putBoolean("revLimit", revLimitStatus());
    kFeederMotor_current = FeederMotor.getSupplyCurrent().getValue();
    SmartDashboard.putNumber("kFeederMotor_current", kFeederMotor_current);

    // This method will be called once per scheduler run
  }
}
