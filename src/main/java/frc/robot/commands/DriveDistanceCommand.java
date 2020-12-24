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
import frc.robot.utils.navigation.*;

public class DriveDistanceCommand extends CommandBase {
  private Node start;
  private Node end;
  private double moe;
  private double distance;
  private SubsystemsInstance inst;
  private boolean finished = false;

  /**
   * Creates a new DriveDistanceCommand.
   */
  public DriveDistanceCommand(Node start, Node end, double moe) {
    inst = SubsystemsInstance.getInstance();
    addRequirements(inst.m_driveSubsystem, inst.m_navSubsystem);

    this.start = start;
    this.end = end;
    this.moe = moe;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing DriveDistance");
    
    distance = start.calcDistance(end) / 0.1467;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving distance");
    
    // convert the distance from feet into revolutions that the encoders measure
    // TODO may have to apply a scale factor for the surface that the robot is driving on
    inst.m_driveSubsystem.movePID(distance, distance, ControlType.kPosition);

    double xErr = (inst.m_driveSubsystem.getKalmanData()[0] - end.getX()) / end.getX();
    double yErr = (inst.m_driveSubsystem.getKalmanData()[1] - end.getY()) / end.getY();

    if(Math.abs(xErr) < moe && Math.abs(yErr) < moe) finished = true;
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
