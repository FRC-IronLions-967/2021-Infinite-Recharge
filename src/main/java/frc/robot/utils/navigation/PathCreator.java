package frc.robot.utils.navigation;

public class PathCreator {
    private Node curNode;
    private Node prevNode;

    public PathCreator() {
        curNode = new Node(0.0, 0.0, NodeType.NAVIGATION);
    }

    
}