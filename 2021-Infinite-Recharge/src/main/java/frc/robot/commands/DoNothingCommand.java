/*
  I give you one guess to figure out what this class.  No points if you get it right.
  This was added as a default safe command for the controller classes to fix the bug
  where they would crash the program if an unassigned button was pressed.
*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingCommand extends CommandBase {
  /**
   * Creates a new DoNothingCommand.
   */
  public DoNothingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}