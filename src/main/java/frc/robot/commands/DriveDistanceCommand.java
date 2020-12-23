/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class DriveDistanceCommand extends CommandBase {
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
    System.out.println("Initializing DriveDistance");
    // inst.m_driveSubsystem.driveDistance(distance);
    // finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving distance");
    
    distance *= 0.1467;
    inst.m_driveSubsystem.movePID(distance, distance, ControlType.kPosition);

    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
