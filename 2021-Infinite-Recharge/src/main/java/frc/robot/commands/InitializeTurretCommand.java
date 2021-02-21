// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class InitializeTurretCommand extends CommandBase {

  private SubsystemsInstance inst;
  private boolean done;  
  /** Creates a new InitializeTurretCommand. */
  public InitializeTurretCommand() {
    inst = SubsystemsInstance.getInstance();
    addRequirements(inst.m_turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inst.m_turretSubsystem.initializeTurret();
    inst.m_turretSubsystem.initializeActuator();
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
