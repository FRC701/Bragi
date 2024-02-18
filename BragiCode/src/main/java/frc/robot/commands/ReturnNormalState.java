// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederEnumState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ReturnNormalState extends InstantCommand {
  /** Creates a new ReturnNormalState. */
  private Feeder mFeeder;

  public ReturnNormalState(Feeder feeder) {
    this.mFeeder = feeder;
    addRequirements(mFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.mShooterState = ShooterState.S_WaitingForFeeder;
    Feeder.mFeederEnumState = FeederEnumState.S_WaitingOnNote;
  }
}
