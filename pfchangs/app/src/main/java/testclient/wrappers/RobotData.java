package testclient.wrappers;

public class RobotData {
    public final VisionData vision;
    public final OdomData odom;

    public class VisionData {
        public final int id;
        public final boolean hasTargets;
        public final int tagID;
        public final double[] distances;
        public final double[] campose;

        public VisionData(int id, boolean hasTargets, int tagID, double[] distances, double[] campose) {
            this.id = id;
            this.hasTargets = hasTargets;
            this.tagID = tagID;
            this.distances = distances;
            this.campose = campose;
        }
    }

    public class OdomData {
        public final int id;
        public final double x;
        public final double y;
        public final double w;

        public OdomData(int id, double x, double y, double w) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.w = w;
        }
    }

    public RobotData(
        int visionID,
        boolean hasTargets,
        int tagID,
        double[] distances,
        double[] campose,
        int odomID,
        double odomX,
        double odomY,
        double odomW
    ) {
        vision = new VisionData(visionID, hasTargets, tagID, distances, campose);
        odom = new OdomData(odomID, odomX, odomY, odomW);
    }
}
