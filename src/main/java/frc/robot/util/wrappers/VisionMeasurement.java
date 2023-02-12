package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionMeasurement {
    private boolean hasTargets;
    private double latency;
    private int tagID = -1;
    private Pose2d botpose;
    private Pose2d campose;
    private double[] distance;
    private double[] distanceX;
    private double[] distanceY;

    public VisionMeasurement() {
        this(
            false,
            -1,
            -1,
            new Pose2d(
                new Translation2d(-1, -1),
                new Rotation2d(-1)
            ),
            new Pose2d(
                new Translation2d(-1, -1),
                new Rotation2d(-1)
            ),
            new double[]{
                -1, -1, -1, -1,
                -1, -1, -1, -1
            },
            new double[]{
                -1, -1, -1, -1,
                -1, -1, -1, -1
            },
            new double[]{
                -1, -1, -1, -1,
                -1, -1, -1, -1
            }
        );
    }

    public VisionMeasurement(
        boolean hasTargets,
        double latency,
        int tagID,
        Pose2d botpose,
        Pose2d campose,
        double[] distance,
        double[] distanceX,
        double[] distanceY
    ) {
        this.hasTargets = hasTargets;
        this.latency = latency;
        this.tagID = tagID;
        this.botpose = botpose;
        this.campose = campose;
        this.distance = distance;
        this.distanceX = distanceX;
        this.distanceY = distanceY;
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

    public double[] getDistanceX() {
        return distanceX;
    }

    public double[] getDistanceY() {
        return distanceY;
    }
}
