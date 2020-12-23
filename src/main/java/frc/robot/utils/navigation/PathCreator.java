package frc.robot.utils.navigation;

/*
    This class will eventually
*/

public class PathCreator {
    private Node curNode;
    private Node prevNode;

    private Node[] samplePath;
    private int nodeIndex = 0;

    public PathCreator() {
        samplePath = new Node[] {new Node(0.0, 0.0, NodeType.NAVIGATION), new Node(3.0, 4.0, NodeType.NAVIGATION), new Node(3.0, 8.0, NodeType.NAVIGATION), new Node(0.0, 12.0, NodeType.NAVIGATION)};

        curNode = samplePath[0];
    }

    public Node getNext() {
        return samplePath[nodeIndex++];
    }   
}