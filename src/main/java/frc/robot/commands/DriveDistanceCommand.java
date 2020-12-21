/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class DriveDistanceCommand extends CommandBase {
  // distance (in feet) to drive
  private double distance;
  private SubsystemsInstance inst;
  private boolean finished = false;
  /**
   * Creates a new DriveDistanceCommand.
   */
  public DriveDistanceCommand(double distance) {
    inst = SubsystemsInstance.getInstance();
    addRequirements(inst.m_driveSubsystem, inst.m_navSubsystem);

    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inst.m_driveSubsystem.driveDistance(distance);
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
