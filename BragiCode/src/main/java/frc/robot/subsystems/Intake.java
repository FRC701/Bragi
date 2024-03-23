// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX IntakeMotor;

  public static double kIntakeMotor_current;
  public static boolean IntakeActive;

  public static IntakeEnumState mIntakeEnumState;

  public Intake() {
    IntakeMotor = new TalonFX(Constants.IntakeConstants.kIntakeMotor, "cani");
    IntakeActive = false;
    mIntakeEnumState = IntakeEnumState.S_WaitingOnNote;
  }

  public enum IntakeEnumState {
    S_WaitingOnNote,
    S_CarryingNote,
    S_IntakeFeed,
    S_Eject
  }

  public void RunIntakeState() {
    switch (mIntakeEnumState) {
      case S_WaitingOnNote:
        WaitingOnNote();
        break;
      case S_CarryingNote:
        CarryingNote();
        break;
      case S_IntakeFeed:
        IntakeFeed();
        break;
      case S_Eject:
        Eject();
        break;
    }
  }
  ;

  public void WaitingOnNote() {
    if (IntakeActive) {
      IntakeMotor.setVoltage(-4);
    } else {
      IntakeMotor.setVoltage(0);
    }
  }

  public void CarryingNote() {
    IntakeMotor.setVoltage(0);
  }

  public void IntakeFeed() {
    IntakeMotor.setVoltage(-4);
  }

  public void Eject() {
    IntakeMotor.setVoltage(6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RunIntakeState();
    SmartDashboard.putBoolean("IntakeActive", IntakeActive);
    SmartDashboard.putString("IntakeState", mIntakeEnumState.toString());
    kIntakeMotor_current = IntakeMotor.getSupplyCurrent().getValue();
    SmartDashboard.putNumber("kIntakeMotor_current", kIntakeMotor_current);
  }
}
