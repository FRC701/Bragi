// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeState;

public class WaitingForBall extends Command {
  private Intake mIntake;
  /** Creates a new IntakeTakeInBall. */
  public WaitingForBall(Intake mIntake) {
    this.mIntake = mIntake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.mIntakeState = IntakeState.S_PopOutIntake;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
