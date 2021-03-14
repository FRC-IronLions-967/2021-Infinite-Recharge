package frc.robot.auto;

import frc.robot.subsystems.*;
import frc.robot.values.*;
import frc.robot.utils.navigation.*;

import java.util.ArrayList;

public class TestAuto implements Autonomous {
    private SubsystemsInstance subsystemsInst;
    private ValuesInstance valInst;
    private ArrayList<Node> path;
    private int curIndex;
    private double tolerance;

    public TestAuto() {
        subsystemsInst = SubsystemsInstance.getInstance();
        valInst = ValuesInstance.getInstance();
        path = new ArrayList<>();

        curIndex = 0;
        tolerance = valInst.m_values.getDoubleValue("autoPosTolerance");
    }

    @Override
    public void init() {
        subsystemsInst.m_driveSubsystem.resetKalman();
        
        path.add(new Node(0.0, 0.0));
        path.add(new Node(5.0, 4.0));
        path.add(new Node(5.0, 8.0));
    }

    @Override
    public void periodic() {
        double x = subsystemsInst.m_driveSubsystem.getX();
        double y = subsystemsInst.m_driveSubsystem.getY();

        Node curLocation = new Node(x, y);

        Node targetNode = path.get(curIndex);

        if(Math.abs(x - targetNode.getX()) < tolerance && Math.abs(y - targetNode.getY()) < tolerance) {
            if(++curIndex < path.size()) targetNode = path.get(curIndex);
        }

        double angle = curLocation.calcAngle(targetNode) - subsystemsInst.m_driveSubsystem.getAngle();
        double distance = curLocation.calcDistance(targetNode);

        // these constants will need to be tuned
        double rPower = 0.08 * distance;
        double lPower = 0.08 * distance;

        rPower -= 0.03 * angle;
        lPower += 0.03 * angle;

        subsystemsInst.m_driveSubsystem.move(rPower, lPower);
    }
}