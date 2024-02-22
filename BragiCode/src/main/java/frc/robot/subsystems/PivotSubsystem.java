// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  private TalonFX mPivotMotor;
  private DutyCycleEncoder mThroughBore;

  public static PivotEnumState mPivotEnum;
  private VisionSubsystem mVisionSubsystem;

  public static double InputAngle = 0;
  public static double SmartAngle = 0;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    mPivotMotor = new TalonFX(PivotConstants.kPivotMotor, "Cani");
    mThroughBore = new DutyCycleEncoder(PivotConstants.kThroughBoreChannel);

    var fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.valueOf(
            (int)
                (mThroughBore.getAbsolutePosition()
                    * PivotConstants.kThroughBoreChannelMultiplier));
    mPivotMotor.getConfigurator().apply(fx_cfg);

    mVisionSubsystem = new VisionSubsystem();

    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = PivotConstants.kF;
    Slot0Configs.kP = PivotConstants.kP;
    Slot0Configs.kI = PivotConstants.kI;
    Slot0Configs.kD = PivotConstants.kD;
    Slot0Configs.kG = PivotConstants.kG;
    Slot0Configs.kS = PivotConstants.kS;

    mPivotMotor.getConfigurator().apply(Slot0Configs);

    mPivotEnum = PivotEnumState.shutoff;
  }

  public enum PivotEnumState {
    S_Fixed,
    S_AgainstSpeaker,
    S_VisionAim,
    shutoff,
    Test
  }

  public void RunPivotState() {
    switch (mPivotEnum) {
      case S_Fixed:
        Fixed();
        break;
      case S_AgainstSpeaker:
        AgainstSpeaker();
      case S_VisionAim:
        VisionAim();
        break;
      case shutoff:
        mPivotMotor.setVoltage(0);
      case Test:
    }
  }

  public void Fixed() {
    // PositionVoltage Pose = new PositionVoltage(DegreesToRawAbsolutePulseOutput(0));
    MotionMagicExpoVoltage Pose = new MotionMagicExpoVoltage(DegreesToRawAbsolutePulseOutput(0));
    mPivotMotor.setControl(Pose);
  }

  public void AgainstSpeaker() {
    // PositionVoltage Pose = new PositionVoltage(DegreesToRawAbsolutePulseOutput(0));
    MotionMagicExpoVoltage Pose = new MotionMagicExpoVoltage(DegreesToRawAbsolutePulseOutput(0));
    mPivotMotor.setControl(Pose);
  }

  public void VisionAim() {
    mPivotMotor.setVoltage(mVisionSubsystem.pivotShooterToTargetOutput());
  }

  public double DegreesToRawAbsolutePulseOutput(double degrees) {
    return degrees * PivotConstants.kThroughBoreChannelMultiplier;
  }

  public void Test() {
    MotionMagicExpoVoltage Pose =
        new MotionMagicExpoVoltage(DegreesToRawAbsolutePulseOutput(SmartAngle));
    mPivotMotor.setControl(Pose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GetABPosition", mThroughBore.getAbsolutePosition());
    SmartDashboard.putNumber("GetRemoteSensor", mPivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("PivotEnumState", mPivotEnum.toString());
    RunPivotState();

    InputAngle = -SmartDashboard.getNumber("Input Angle", 0);

    // This method will be called once per scheduler run
  }
}
