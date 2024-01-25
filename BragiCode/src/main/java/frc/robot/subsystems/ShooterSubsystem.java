// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Feeder.FeederEnumState;
import frc.robot.subsystems.Intake.IntakeState;


public class ShooterSubsystem extends SubsystemBase {
  public TalonFX ShooterMotorLeft;
  public TalonFX ShooterMotorRight;
  

  public ShooterState mShooterState;

  private Timer mTimer = new Timer();

  public final Joystick mDriver = new Joystick(OperatorConstants.kDriverControllerPort);

  private Feeder mFeeder = new Feeder();
  private Intake mIntake = new Intake();

  public enum ShooterState {
  S_Shoot, S_WaitingForFeeder
}

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    ShooterMotorLeft = new TalonFX(Constants.ShooterConstants.kShooterMotorLeft);
    ShooterMotorRight = new TalonFX(Constants.ShooterConstants.kShooterMotorRight);

    ShooterMotorRight.setControl(new Follower(ShooterMotorLeft.getDeviceID(), Constants.kOpposeMasterDirection));


  }

  public void RunShooterState(){
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
      ShooterMotorLeft.set(0.5);
    }

    public void Shoot() {
      ShooterMotorLeft.set(DoubleSupplier(mDriver).Velocity);
      mTimer.start();
      if(mTimer.hasElapsed(5)){
        mTimer.stop();
        mTimer.reset();
        mIntake.mIntakeState = IntakeState.S_WaitingForBall;
        mFeeder.mFeederEnumState = FeederEnumState.S_WaitingOnIntake;
        mShooterState = ShooterState.S_WaitingForFeeder;
      }
    }

    public VelocityDutyCycle DoubleSupplier(Joystick joystick){
      DoubleSupplier Speed = ()-> (joystick.getThrottle() + 1)/2 * 6380;
      VelocityDutyCycle mSpeed = new VelocityDutyCycle(Speed.getAsDouble());
      return mSpeed;
    }

    private static double ShooterVelo(TalonFX motorFx){
      return motorFx.getVelocity().getValueAsDouble();
    }
  
    public void Velocity(){
      SmartDashboard.putNumber("ShooterMotorRight",ShooterVelo(ShooterMotorRight));
      SmartDashboard.putNumber("ShooterMotorLeft",ShooterVelo(ShooterMotorLeft));
    }
  @Override
  public void periodic() {
    RunShooterState();
    Velocity();
    // This method will be called once per scheduler run
  }
}
