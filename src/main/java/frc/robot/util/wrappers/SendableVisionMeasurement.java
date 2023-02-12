package frc.robot.util.wrappers;

public class SendableVisionMeasurement {
    private int measID;
    private boolean hasTargets;
    private int tagID;
    private double[] distance;
    private double[] distanceX;
    private double[] distanceY;
    private double[] campose;

    public SendableVisionMeasurement(int measID) {
        this(measID, 
            false, 
            -1, 
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
            },
            new double[]{
                -1, -1, -1,
                -1, -1, -1
            }
        );
    }

    public SendableVisionMeasurement(
        int measID, 
        boolean hasTargets, 
        int tagID, 
        double[] distance, 
        double[] distanceX,
        double[] distanceY,
        double[] campose
    ) {
        this.measID = measID;
        this.hasTargets = hasTargets;
        this.tagID = tagID;
        this.distance = distance;
        this.distanceX = distanceX;
        this.distanceY = distanceY;
        this.campose = campose;
    }

    public void setMeasID(int id) {
        measID = id;
    }

    public int getMeasID() {
        return measID;
    }

    public boolean hasTargets() {
        return hasTargets;
    }

    public int getTagID() {
        return tagID;
    }

    public double[] getDistance() {
        return distance;
    }

    public double[] getCamPose() {
        return campose;
    }

    public double[] getDistanceX() {
        return distanceX;
    }

    public double[] getDistanceY() {
        return distanceY;
    }
}
