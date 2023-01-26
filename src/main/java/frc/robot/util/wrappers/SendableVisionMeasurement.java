package frc.robot.util.wrappers;

public class SendableVisionMeasurement {
    private int measID;
    private boolean hasTargets;
    private int tagID;
    private double[] distance;

    public SendableVisionMeasurement(int measID, boolean hasTargets, int tagID, double[] distance) {
        this.measID = measID;
        this.hasTargets = hasTargets;
        this.tagID = tagID;
        this.distance = distance;
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

}
