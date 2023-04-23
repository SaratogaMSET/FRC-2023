package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Pose2d;

public class FilterEstimate {
    private int id;
    private Pose2d pose;

    public FilterEstimate(int id, Pose2d pose) {
        this.id = id;
        this.pose = pose;
    }

    public int getID() {
        return id;
    }

    public Pose2d getPose() {
        return pose;
    }
}
