package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    private boolean hasTargets;
    private double latency;
    private int tagID = -1;
    private Pose2d botpose;
    private Pose2d campose;
    private double[] distance;

    public VisionMeasurement(
        boolean hasTargets,
        double latency,
        int tagID,
        Pose2d botpose,
        Pose2d campose,
        double... distance
    ) {
        this.hasTargets = hasTargets;
        this.latency = latency;
        this.tagID = tagID;
        this.botpose = botpose;
        this.campose = campose;
        this.distance = distance;
    }

    public boolean hasTargets() {
        return hasTargets;
    }
    
    public double getLatency() {
        return latency;
    }
    
    public int getTagID() {
        return tagID;
    }
    
    public Pose2d getBotpose() {
        return botpose;
    }

    public Pose2d getCampose() {
        return campose;
    }
    
    public double[] getDistance() {
        return distance;
    }
}
