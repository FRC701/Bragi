// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor1;

  private TalonFX FeederMotor2;
  public FeederEnumState mFeederEnumState;

  public enum FeederEnumState {
    S_WaitingOnIntake,
    S_DriverReady
  }

  public Feeder() {
    FeederMotor1 = new TalonFX(Constants.FeederConstants.kFeederMotor1);
    FeederMotor2 = new TalonFX(Constants.FeederConstants.kFeederMotor2);
    FeederMotor2.setControl(new Follower(Constants.FeederConstants.kFeederMotor1, false));
    mFeederEnumState = FeederEnumState.S_WaitingOnIntake;
  }

  public void RunFeederState() {
    switch (mFeederEnumState) {
      case S_WaitingOnIntake:
        WaitingOnIntake();
        break;
      case S_DriverReady:
        DriverReady();
        break;
    }
  }

  public void WaitingOnIntake() {
    FeederMotor1.stopMotor();
  }

  public void DriverReady() {
    FeederMotor1.set(0.1);
    // Need shoot command and shooter subsystem to be done
    // wait for shooter to become ready
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
