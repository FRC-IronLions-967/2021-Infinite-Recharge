/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import frc.robot.utils.navigation.*;
import frc.robot.commands.*;

public class NavigationSubsystem extends SubsystemBase {
  private boolean navEnabled = false;
  
  private Node curNode;
  private ArrayList<Node> destinations;
  private int nodeIndex = 0;

  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {
    destinations = new ArrayList<>();
    curNode = new Node(0.0, 0.0, NodeType.NAVIGATION);
  }

  public boolean isNavEnabled() {
    return navEnabled;
  }

  public void setNavEnabled(boolean enabled) {
    navEnabled = enabled;
  }

  public void addDestination(Node dest) {
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
        CommandScheduler.getInstance().schedule(new TurnToAngleCommand(curNode.calcAngle(destinations.get(nodeIndex))));
        CommandScheduler.getInstance().schedule(new DriveDistanceCommand(curNode.calcDistance(destinations.getNode(nodeIndex))));
        curNode = destinations.get(nodeIndex);

        nodeIndex++;
      }
    }
  }
}
