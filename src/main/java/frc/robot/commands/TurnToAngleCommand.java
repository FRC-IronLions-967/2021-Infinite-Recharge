/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemsInstance;

public class TurnToAngleCommand extends CommandBase {

  // the angle in radians to turn to (absolute, does not depend on current angle)
  private double angle;
  private SubsystemsInstance inst;
  private boolean finished = false;
  /**
   * Creates a new TurnToAngleCommand.
   */
  public TurnToAngleCommand(double angle) {
    inst = SubsystemsInstance.getInstance();
    addRequirements(inst.m_driveSubsystem, inst.m_navSubsystem);

    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing TurnToAngle");
    // inst.m_driveSubsystem.turnToAngle(angle);
    // finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running turn to angle");
    // proprotional constant to multiply p by
    double p = 1.0e-3;
    // margin of error in revolutions
    double moe = 0.02;
    double theta = inst.m_driveSubsystem.getGyroAngle() * (Math.PI/180.0);
    double error = (theta - angle) / (2.0 * Math.PI);;

    while(Math.abs(error) > moe) {
      // if error is positive, this will cause the robot to rotate clockwise, decreasing the angle
      // if error is negative, this will cause the robot to rotate counterclockwise, increasing the angle
      System.out.println("Error: " + error);
      System.out.println("Angle: " + angle);
      System.out.println("Theta: " + theta);
      // rightMaster.set(-p * error);
      // leftMaster.set(p * error);
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
