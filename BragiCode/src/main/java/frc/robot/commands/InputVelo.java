// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InputVelo extends InstantCommand {
  @SuppressWarnings("unused")
  private ShooterSubsystem mShooterSubsystem;

  public InputVelo(ShooterSubsystem mShooterSubsystem) {
    this.mShooterSubsystem = mShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem);
  }

  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.mSmartSpeed = 0;
    ShooterSubsystem.mShooterState = ShooterState.S_AccelerateShooter;
    ShooterSubsystem.mSmartSpeed = ShooterSubsystem.InputVelocity;
  }
}
