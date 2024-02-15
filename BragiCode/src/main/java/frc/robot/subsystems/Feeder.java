// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor;

  public static FeederEnumState mFeederEnumState;

  private TalonFXConfiguration mTalonFXConfig;

  private static Timer Timer;

  public Feeder() {
    FeederMotor = new TalonFX(Constants.FeederConstants.kFeederMotor1);
    mFeederEnumState = FeederEnumState.S_WaitingOnNote;
    Timer = new Timer();
    mTalonFXConfig = new TalonFXConfiguration();
    mTalonFXConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    FeederMotor.getConfigurator().apply(mTalonFXConfig);
  }

  public enum FeederEnumState {
    S_WaitingOnNote,
    S_NoteInIntake,
    S_ShooterReady,
    S_funEject;
  }

  public void RunFeederState() {
    switch (mFeederEnumState) {
      case S_WaitingOnNote:
        WaitingOnNote();
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

  public void WaitingOnNote() {
    if (!revLimitStatus()) {
      mFeederEnumState = FeederEnumState.S_NoteInIntake;
    } else {
      FeederMotor.set(-0.25);
    }
  }

  public void NoteInIntake() {
    FeederMotor.set(0);
  }

  public void ShooterReady() {
    FeederMotor.set(-0.25);
    // Need shoot command and shooter subsystem to be done
    // wait for shooter to become ready
  }

  public void funEject() {
    if (Timer.hasElapsed(0.5)) {
      Timer.stop();
      Timer.reset();
      mFeederEnumState = FeederEnumState.S_WaitingOnNote;
    } else {
      Timer.start();
      FeederMotor.set(0.5);
    }
  }

  public boolean revLimitStatus() {
    return (FeederMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("revLimit", revLimitStatus());
    SmartDashboard.putString("FeederState", mFeederEnumState.toString());
    RunFeederState();

    // This method will be called once per scheduler run
  }
}
