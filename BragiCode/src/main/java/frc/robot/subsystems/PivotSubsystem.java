// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

  private PIDController mPIDcontroller;
  private ArmFeedforward mFFcontroller;
  public static double kPivotMotor_current;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    mPivotMotor = new TalonFX(PivotConstants.kPivotMotor);
    mThroughBore = new DutyCycleEncoder(PivotConstants.kThroughBoreChannel);

    mPIDcontroller = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);

    var fx_cfg = new MotorOutputConfigs();

    fx_cfg.NeutralMode = NeutralModeValue.Brake;

    mPivotMotor.getConfigurator().apply(fx_cfg);
    mFFcontroller = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

    // mPIDcontroller.setIntegratorRange(-12, 12);

    /*  var fx_cfg = new TalonFXConfiguration();
    fx_cfg.DifferentialSensors.DifferentialSensorSource = DifferentialSensorSourceValue.
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.
        //FeedbackSensorSourceValue.valueOf((int) ((mThroughBore.getAbsolutePosition() * 180) + 180 + 1000000));
    mPivotMotor.getConfigurator().apply(fx_cfg); */

    mVisionSubsystem = new VisionSubsystem();

    /*var Slot0Configs = new Slot0Configs();
    Slot0Configs.kV = PivotConstants.kF;
    Slot0Configs.kP = PivotConstants.kP;
    Slot0Configs.kI = PivotConstants.kI;
    Slot0Configs.kD = PivotConstants.kD;
    Slot0Configs.kG = PivotConstants.kG;
    Slot0Configs.kS = PivotConstants.kS;

    mPivotMotor.getConfigurator().apply(Slot0Configs);*/

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
        break;
      case S_VisionAim:
        VisionAim();
        break;
      case shutoff:
        mPivotMotor.setVoltage(0);
        break;
      case Test:
        Test();
        break;
    }
  }

  public void Fixed() {
    // PositionVoltage Pose = new PositionVoltage(DegreesToRawAbsolutePulseOutput(0));
    // MotionMagicExpoVoltage Pose = new MotionMagicExpoVoltage(DegreesToRawAbsolutePulseOutput(0));
    double Output = Output(62);
    mPivotMotor.setVoltage(Output);
  }

  public void AgainstSpeaker() {
    // PositionVoltage Pose = new PositionVoltage(DegreesToRawAbsolutePulseOutput(0));
    // MotionMagicExpoVoltage Pose = new MotionMagicExpoVoltage(DegreesToRawAbsolutePulseOutput(0));
    double Output = Output(40);
    mPivotMotor.setVoltage(Output);
    // SmartDashboard.putNumber("work", Output(SmartAngle));

  }

  public void VisionAim() {

    double Output = Output(mVisionSubsystem.pivotShooterToTargetOutput());
    mPivotMotor.setVoltage(MathUtil.applyDeadband(Output, 0.05));
  }

  public void Test() {
    double Output = Output(SmartAngle);
    mPivotMotor.setVoltage(Output);
  }

  public double Output(double Setpoint) {
    double nextOutput =
        mFFcontroller.calculate(ABSposition(), Setpoint)
            + mPIDcontroller.calculate(ABSposition(), Setpoint);

    // SlewRateLimiter m_slew = new SlewRateLimiter(0.1);

    return /*m_slew.calculate(-nextOutput) */ -nextOutput;
  }

  public double ABSposition() {

    return (((mThroughBore.getAbsolutePosition() * 180) + 180) - PivotConstants.kEncoderToZero)
            * (PivotConstants.kEncoderRange / PivotConstants.kEncoderUpperBound)
        + PivotConstants.kEncoderOffset;
  }

  public boolean fwdLimitSwitch() {
    return mPivotMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean revLimitSwitch() {
    return mPivotMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GetABPosition", ABSposition());
    // SmartDashboard.putNumber("GetRemoteSensor", mPivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("PivotEnumState", mPivotEnum.toString());

    SmartDashboard.putBoolean("fwd", fwdLimitSwitch());
    SmartDashboard.putBoolean("rev", revLimitSwitch());

    SmartDashboard.putNumber("SmartAngle", SmartAngle);

    SmartDashboard.putNumber("Out", Output(SmartAngle));

    SmartDashboard.putNumber("DesiredVisionAngle", mVisionSubsystem.pivotShooterToTargetOutput());

    RunPivotState();

    InputAngle = SmartDashboard.getNumber("Input Angle", 0);
    // This method will be called once per scheduler run
    kPivotMotor_current = mPivotMotor.getSupplyCurrent().getValue();
    SmartDashboard.putNumber("kPivotMotor_current", kPivotMotor_current);
  }
}
