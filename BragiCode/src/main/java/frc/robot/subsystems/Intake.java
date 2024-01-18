// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public TalonFX IntakeM1;
  public TalonFX IntakeM2;
  public TalonFX IntakeM3;
  public IntakeState mIntakeState;

  /** Creates a new Intake. */

  // State Machine
  public enum IntakeState {
    S_WaitingForBall, S_PopOutIntake, S_WaitingForShooter
  }

  public Intake() {
    IntakeM1 = new TalonFX(Constants.IntakeConstants.IntakeMotor1);
    IntakeM2 = new TalonFX(Constants.IntakeConstants.IntakeMotor2);
    IntakeM3 = new TalonFX(Constants.IntakeConstants.IntakeMotor3);

    mIntakeState = IntakeState.S_WaitingForBall;
  }

  public void RunIntakeState() {
    switch (mIntakeState) {
      case S_WaitingForBall:
        WaitingForBall();
        break;
      case S_PopOutIntake:

        break;
      case S_WaitingForShooter:
        WaitingForShooter();
        break;
    }
  }

  public void WaitingForBall() {
    IntakeM1.set(0.1);
    IntakeM2.set(0.1);
   ]
  }

  public void PopOutIntake() {
    IntakeM3.set(0.8);
      if (IntakeM1.getFault_ForwardHardLimit().getValue()) {
      mIntakeState = IntakeState.S_WaitingForShooter;
    }    
  }

  public void WaitingForShooter() {
    IntakeM1.set(0);
    IntakeM2.set(0);

  }

  @Override
  public void periodic() {
    RunIntakeState();
    // This method will be called once per scheduler run
  }
}
