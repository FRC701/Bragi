// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
  public TalonFX ShooterMotorLeft;
  public TalonFX ShooterMotorRight;

  public ShooterState mShooterState;

  public enum ShooterState{
  S_Shoot, S_WaitingForFeeder
};
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    ShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    ShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

  }

  public void RunShooter(){
      switch (mShooterState){
        case S_WaitingForFeeder:
          WaitingForFeeder();
          break;  
        case S_Shoot:
          Shoot();
          break;
      }

    }

    public void WaitingForFeeder(){

    }

    public void Shoot(){
<<<<<<< Updated upstream

=======
      
>>>>>>> Stashed changes
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
