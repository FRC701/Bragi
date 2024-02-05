// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Generated.TunerConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stop extends InstantCommand {
  private CommandSwerveDrivetrain mDrivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(6 * 0.1)
          .withRotationalDeadband(1.5 * Math.PI * 0.28) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public Stop() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  } // !FieldRelativeMayBeWrong!
}
