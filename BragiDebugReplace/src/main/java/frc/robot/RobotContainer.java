// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final Feeder mFeeder = new Feeder();
  private final ShooterSubsystem mShooter = new ShooterSubsystem();
  // measure pdb power and temp data
  private final PowerSubsystem m_pdb = new PowerSubsystem();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
