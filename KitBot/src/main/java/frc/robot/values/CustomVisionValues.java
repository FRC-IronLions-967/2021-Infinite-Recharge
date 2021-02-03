package frc.robot.values;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CustomVisionValues {

    private NetworkTableInstance inst;

    // NetworkTable that contains all of the subtables that track individual pipelines
    private NetworkTable visionTable;

    // subtable that tracks this pipeline
    private NetworkTable pipelineTable;

    // entries for values to be tracked
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry width;
    private NetworkTableEntry height;
    private NetworkTableEntry area;
    private NetworkTableEntry dist;
    private NetworkTableEntry angleOffset;
    private NetworkTableEntry reliability;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pipelineName;

    // creates a new object that pulls data from the specified pipeline
    public CustomVisionValues(String pipeline) {
        inst = NetworkTableInstance.getDefault();

        visionTable = inst.getTable("vision");
        if(visionTable.containsSubTable(pipeline))
            pipelineTable = visionTable.getSubTable(pipeline);
        // TODO put in an exception to be thrown if it cannot find the pipeline

        tx = pipelineTable.getEntry("tx");
        ty = pipelineTable.getEntry("ty");
        width = pipelineTable.getEntry("width");
        height = pipelineTable.getEntry("height");
        area = pipelineTable.getEntry("area");
        dist = pipelineTable.getEntry("dist");
        angleOffset = pipelineTable.getEntry("angleOffset");
        reliability = pipelineTable.getEntry("reliability");
        hasTarget = pipelineTable.getEntry("hasTarget");
        pipelineName = pipelineTable.getEntry("pipelineName");
    }

    // returns the value of tx, or the maximum value for a double if it can't get the value
    public double getTX() {
        return tx.getDouble(Double.MAX_VALUE);
    }

    // returns the value of ty, or the maximum value for a double if it can't get the value
    public double getTY() {
        return ty.getDouble(Double.MAX_VALUE);
    }

    // returns the width of the bounding rectangle around the found vision target, or 0.0 if the bounding rectangle isn't present
    public double getWidth() {
        return width.getDouble(0.0);
    }

    // returns the height of the bounding rectangle around the found vision target, or 0.0 if the bounding rectangle isn't present
    public double getHeight() {
        return height.getDouble(0.0);
    }

    // returns the area of the bounding rectangle around the found vision target, or 0.0 if the bounding rectangle isn't present
    public double getArea() {
        return area.getDouble(0.0);
    }

    // returns the estimated distance from the robot to the target based on the dimensions of the vision target, or the maximum
    // value for a double if it can't estimate the distance, can't find the target, or can't retrieve the value
    public double getDist() {
        return dist.getDouble(Double.MAX_VALUE);
    }

    // returns the estimated angle between the robot's current position and being pointed straight at the vision target, or the
    // maximum value for a double if it can't estimate the angle, can't find the target, or can't retrieve the value
    public double getAngleOffset() {
        return angleOffset.getDouble(Double.MAX_VALUE);
    }

    // returns the reliability reported by the pipeline, or 0.0 if it cannot retrieve the value or the pipeline has no target
    public double getReliability() {
        return reliability.getDouble(0.0);
    }

    // returns true if the vision pipeline is currently tracking a target within its FOV, false otherwise
    public boolean hasTarget() {
        return hasTarget.getBoolean(false);
    }

    // returns the name of this pipeline or "Unknown Pipeline" if it cannot find the name
    public String getPipelineName() {
        return pipelineName.getString("Unknown Pipeline");
    }
}