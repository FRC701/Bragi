// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor;

  public FeederEnumState mFeederEnumState;

  public enum FeederEnumState {
    S_WaitingOnNote,
    S_NoteInIntake,
    S_ShooterReady
  }

  public Feeder() {
    FeederMotor = new TalonFX(Constants.FeederConstants.kFeederMotor1);
    mFeederEnumState = FeederEnumState.S_WaitingOnNote;
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
    FeederMotor.set(0.1);
    if (FeederMotor.getFault_ForwardHardLimit().getValue()) {
      mFeederEnumState = FeederEnumState.S_NoteInIntake;
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

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("BannerSensor", FeederMotor.getFault_ForwardHardLimit().getValue());
    SmartDashboard.putString("FeederState", "=" + mFeederEnumState);
    RunFeederState();
    // This method will be called once per scheduler run
  }
}
