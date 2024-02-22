// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ActivateElevator extends Command {
  private Elevator mElevator;
  private DoubleSupplier mSpeed;

  /** Creates a new ActivateElevator. */
  public ActivateElevator(Elevator mElevator, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mElevator = mElevator;
    this.mSpeed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mElevator.MoveElevator(mSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mElevator.MoveElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
