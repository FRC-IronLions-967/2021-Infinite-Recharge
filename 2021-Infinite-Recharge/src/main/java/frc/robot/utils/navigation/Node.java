package frc.robot.utils.navigation;

public class Node {
    // coordinate location of this node
    private double x;
    private double y;

    public Node(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double calcAngle(Node otherNode) {
        return Math.atan2(otherNode.getY() - y, otherNode.getX() - x);
    }

    public double calcDistance(Node otherNode) {
        return Math.sqrt(Math.pow(otherNode.getX() - x, 2) + Math.pow(otherNode.getY() - y, 2));
    }
}