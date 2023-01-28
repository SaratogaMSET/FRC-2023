package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Pose2d;

public class SendableOdomMeasurement {
    private int id;
    private Pose2d pose;

    public SendableOdomMeasurement(int id, Pose2d pose) {
        this.id = id;
        this.pose = pose;
    }

    public void setID(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public Pose2d getPose() {
        return pose;
    }
}
