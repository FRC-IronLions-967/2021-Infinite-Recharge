/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;
import frc.robot.utils.navigation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngleCommand extends CommandBase {
  private Node start;
  private Node end;
  private double angle;
  private double moe;
  private SubsystemsInstance inst;
  private boolean finished = false;
  /**
   * Creates a new TurnToAngleCommand.
   */
  public TurnToAngleCommand(Node start, Node end, double moe) {
    inst = SubsystemsInstance.getInstance();
    addRequirements(inst.m_driveSubsystem, inst.m_navSubsystem);

    this.start = start;
    this.end = end;
    this.moe = moe;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing TurnToAngle");

    angle = start.calcAngle(end);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running turn to angle");
    // proprotional constant to multiply p by
    double p = 1.0e-3;
    // margin of error in revolutions
    double theta = inst.m_driveSubsystem.getGyroAngle() * (Math.PI/180.0);
    double error = (theta - angle) / (2.0 * Math.PI);;

    while(Math.abs(error) > moe) {
      // if error is positive, this will cause the robot to rotate clockwise, decreasing the angle
      // if error is negative, this will cause the robot to rotate counterclockwise, increasing the angle
      SmartDashboard.putNumber("Nav Angle Error", error);
      SmartDashboard.putNumber("Nav Angle", angle);

      inst.m_driveSubsystem.move(-p * error, p * error);

      theta = inst.m_driveSubsystem.getGyroAngle() * (Math.PI/180.0);
      error = (theta - angle) / (2.0 * Math.PI);
    }

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
