// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor;

  public FeederEnumState mFeederEnumState;

  public Feeder() {
    FeederMotor = new TalonFX(24);
    mFeederEnumState = FeederEnumState.S_WaitingOnNote;
  }

  public enum FeederEnumState {
    S_WaitingOnNote,
    S_NoteInIntake,
    S_ShooterReady
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
    }
  }

  public void WaitingOnNote() {
    if (revLimitStatus()) {
      mFeederEnumState = FeederEnumState.S_NoteInIntake;
    }
    else{
      FeederMotor.set(0.1);
    }
  }

  public void NoteInIntake() {
    FeederMotor.set(0);
  }

  public void ShooterReady() {
    FeederMotor.set(0.1);
    // Need shoot command and shooter subsystem to be done
    // wait for shooter to become ready
  }

  public boolean revLimitStatus()
  {
    return (FeederMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("revLimit", revLimitStatus());
    SmartDashboard.putString("FeederState", mFeederEnumState.toString());
    RunFeederState();

    // This method will be called once per scheduler run
  }
}
