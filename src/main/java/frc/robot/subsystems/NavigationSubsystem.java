/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import frc.robot.utils.navigation.*;
import frc.robot.commands.*;

public class NavigationSubsystem extends SubsystemBase {
  private boolean navEnabled = false;
  
  private Node curNode;
  private ArrayList<Node> destinations;
  private int nodeIndex = 0;

  private SubsystemsInstance inst;

  private TurnToAngleCommand turn;
  private DriveDistanceCommand drive;

  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {
    destinations = new ArrayList<>();
    curNode = new Node(0.0, 0.0, NodeType.NAVIGATION);

    // inst = SubsystemsInstance.getInstance();
  }

  // public void init() {
  //   inst = SubsystemsInstance.getInstance();
  // }

  public boolean isNavEnabled() {
    return navEnabled;
  }

  public void setNavEnabled(boolean enabled) {
    turn = new TurnToAngleCommand(0.0);
    drive = new DriveDistanceCommand(0.0);
    navEnabled = enabled;
  }

  public void addDestination(Node dest) {
    System.out.println("Added new node");
    destinations.add(dest);
  }

  // returns the nodes previously visited, in the order they were visited
  public Node[] getTraveledPath() {
    Node path[] =  new Node[nodeIndex];
    for(int i = 0; i < nodeIndex; i++) path[i] = destinations.get(i);

    return path;
  }

  public Node getCurNode() {
    return curNode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(navEnabled) {
      if(destinations.size() > nodeIndex) {
        // System.out.println("Navigating to new node");
        // inst.m_driveSubsystem.turnToAngle(curNode.calcAngle(destinations.get(nodeIndex)));
        if(!CommandScheduler.getInstance().isScheduled(turn)) {
          turn = new TurnToAngleCommand(curNode.calcAngle(destinations.get(nodeIndex)));
        }
        CommandScheduler.getInstance().schedule(false, turn);
        // while(!turn.isFinished());
        // System.out.println("Finished turn");
        // inst.m_driveSubsystem.driveDistance(curNode.calcDistance(destinations.get(nodeIndex)));
        if(!CommandScheduler.getInstance().isScheduled(turn)) {
          drive = new DriveDistanceCommand(curNode.calcDistance(destinations.get(nodeIndex)));
        }
        CommandScheduler.getInstance().schedule(false, drive);
        // while(!drive.isFinished());
        // System.out.println("Finished driving");
        if(drive.isFinished() && turn.isFinished()) {
          curNode = destinations.get(nodeIndex);

          nodeIndex++;
        }
      }
    }
  }
}
