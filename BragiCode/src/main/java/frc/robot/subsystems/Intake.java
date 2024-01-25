// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public TalonFX IntakeMotorRight;
  public TalonFX IntakeMotorLeft;
  public TalonFX IntakePopMotor;
  public IntakeState mIntakeState;

  /** Creates a new Intake. */

  // State Machine
  public enum IntakeState {
    S_WaitingForBall, S_PopOutIntake, S_WaitingForShooter
  }

  public Intake() {
    IntakeMotorRight = new TalonFX(Constants.IntakeConstants.kIntakeMotor1);
    IntakeMotorLeft = new TalonFX(Constants.IntakeConstants.kIntakeMotor2);
    IntakePopMotor = new TalonFX(Constants.IntakeConstants.kIntakeMotor3);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0;
    IntakePopMotor.getConfigurator().apply(slot0Configs, 0.050);

    mIntakeState = IntakeState.S_WaitingForBall;
  }

  public void RunIntakeState() {
    switch (mIntakeState) {
      case S_WaitingForBall:
        WaitingForBall();
        break;
      case S_PopOutIntake:
        PopOutIntake();
        break;
      case S_WaitingForShooter:
        WaitingForShooter();
        break;
    }
  }

  public void WaitingForBall() {
    IntakeMotorRight.set(0.1);
    IntakeMotorLeft.set(0.1);

  }

  public void PopOutIntake() {
    IntakePopMotor.setPosition(0);
    if (IntakeMotorRight.getFault_ForwardHardLimit().getValue()) {
      mIntakeState = IntakeState.S_WaitingForShooter;
    }
  }

  public void WaitingForShooter() {
    IntakeMotorRight.set(0);
    IntakeMotorLeft.set(0);
  }

  private static double IntakePos(TalonFX motorFx) {
    return motorFx.getPosition().getValueAsDouble();
  }

  public void Position() {
    SmartDashboard.putNumber("Intake Right Motor Position", IntakePos(IntakeMotorRight));
    SmartDashboard.putNumber("Intake Left Motor Position", IntakePos(IntakeMotorLeft));
    SmartDashboard.putNumber("Intake Motor Pop Position", IntakePos(IntakePopMotor));
  }

  @Override
  public void periodic() {
    RunIntakeState();
    Position();

    // This method will be called once per scheduler run
  }
}
