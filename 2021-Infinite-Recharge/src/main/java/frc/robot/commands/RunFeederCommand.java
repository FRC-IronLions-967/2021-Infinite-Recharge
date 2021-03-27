// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class RunFeederCommand extends CommandBase {

  private double speed;
  private SubsystemsInstance subsystemsInst;

  /** Creates a new RunFeederCommand. */
  public RunFeederCommand(double speed) {
    subsystemsInst = SubsystemsInstance.getInstance();
    addRequirements(subsystemsInst.m_shooterSubsystem);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystemsInst.m_shooterSubsystem.runFeeder(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
