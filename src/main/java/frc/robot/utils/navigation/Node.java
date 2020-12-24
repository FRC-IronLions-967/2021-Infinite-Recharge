package frc.robot.utils.navigation;

public class Node {
    private double x;
    private double y;
    private NodeType type;

    public Node(double x, double y, NodeType type) {
        this.x = x;
        this.y = y;
        this.type = type;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public NodeType getType() {
        return type;
    }

    // calculates the distance between this node and otherNode
    public double calcDistance(Node otherNode) {
        double dx = Math.pow(x - otherNode.getX(), 2);
        double dy = Math.pow(y - otherNode.getY(), 2);

        return Math.sqrt(dx + dy);
    }

    // returns the angle in radians of the line relative to the x-axis that intersects both points
    public double calcAngle(Node otherNode) {
        double dx = x - otherNode.getX();
        double dy = y - otherNode.getY();

        // these arguments might need to be reversed if this gives wrong angle measures
        return Math.atan2(dy, dx);
    }
}