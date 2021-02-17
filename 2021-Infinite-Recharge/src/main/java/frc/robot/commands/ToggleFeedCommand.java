// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class ToggleFeedCommand extends CommandBase {

  private SubsystemsInstance subsystemsInst;
  private boolean done = false;

  /** Creates a new ToggleFeedCommand. */
  public ToggleFeedCommand() {
    subsystemsInst = SubsystemsInstance.getInstance();
    addRequirements(subsystemsInst.m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystemsInst.m_shooterSubsystem.toggleFeeder();
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
