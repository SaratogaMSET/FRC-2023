package testclient;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import testclient.filter.AMCL;
import testclient.wrappers.RobotData;

public class FishClientNT {
    private AMCL amcl = new AMCL();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public final NetworkTable limelightTable = inst.getTable("limelight"); // this is so scuffed aaaaaaa

    private final NetworkTable visionTable = inst.getTable("vision");
    private final IntegerSubscriber visionIDSub = visionTable.getIntegerTopic("id").subscribe(-1);
    private final BooleanSubscriber hasTargetsSub = visionTable.getBooleanTopic("hasTargets").subscribe(false);
    private final IntegerSubscriber tagIDSub = visionTable.getIntegerTopic("tagID").subscribe(-1);
    private final DoubleArraySubscriber distanceSub = visionTable.getDoubleArrayTopic("distances").subscribe(
        new double[]{-1, -1, -1, -1, -1, -1, -1, -1});
    private final DoubleArraySubscriber camposeSub = visionTable.getDoubleArrayTopic("campose").subscribe(
        new double[3]);

    private final NetworkTable odomTable = inst.getTable("odom");
    private final IntegerSubscriber odomIDSub = odomTable.getIntegerTopic("id").subscribe(-1);
    private final DoubleSubscriber odomXSub = odomTable.getDoubleTopic("x").subscribe(-1);
    private final DoubleSubscriber odomYSub = odomTable.getDoubleTopic("y").subscribe(-1);
    private final DoubleSubscriber odomWSub = odomTable.getDoubleTopic("w").subscribe(-1);

    private final NetworkTable estimateTable = inst.getTable("estimated");
    private final IntegerPublisher estimateIDPub = estimateTable.getIntegerTopic("id").publish();
    private final DoublePublisher estimateXPub = estimateTable.getDoubleTopic("x").publish();
    private final DoublePublisher estimateYPub = estimateTable.getDoubleTopic("y").publish();
    private final DoublePublisher estimateWPub = estimateTable.getDoubleTopic("w").publish();

    private final IntegerSubscriber resetFlagSub = odomTable.getIntegerTopic("flag").subscribe(0);
    private int resetFlag = 0;
    private int prevFlag = resetFlag;

    private Pose2d prevOdomPose = new Pose2d();
    private Pose2d currentOdomPose = new Pose2d();
    private Pose2d poseDeltas = new Pose2d();

    public FishClientNT() {
        inst.startClient4("estimator");
        inst.setServerTeam(649);
        // inst.startDSClient();
        // inst.setServer("localhost"); // "localhost" for simulation
        // https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html
        System.out.println("Finshed client init.");

        System.out.println("Initializing AMCL.");
        amcl.init();
        System.out.println("Finished initializing AMCL.");
    }

    private RobotData readNTData() {
        return new RobotData(
            (int) visionIDSub.get(), 
            hasTargetsSub.get(), 
            (int) tagIDSub.get(), 
            distanceSub.get(), 
            camposeSub.get(),
            (int) odomIDSub.get(),
            odomXSub.get(),
            odomYSub.get(),
            odomWSub.get()
        );
    }

    private void publishEstimate(int id, Pose2d pose) {
        estimateIDPub.set(id);
        estimateXPub.set(pose.getX());
        estimateYPub.set(pose.getY());
        estimateWPub.set(pose.getRotation().getRadians());
    }

    public void startAMCL() {
        System.out.println("Starting AMCL.");
        while (odomIDSub.get() == -1) {}
        System.out.println("Received first measurement!");

        amcl.resetMCL();
        while (true) {
            prevFlag = resetFlag;
            resetFlag = (int) resetFlagSub.get();
            if (prevFlag != resetFlag) {
                System.out.println("REINITIALIZING FILTER!!!!!!!!!!!");
                amcl.resetMCL();
            }
            if (odomIDSub.get() != -1) {
                RobotData latestData = readNTData();
                prevOdomPose = currentOdomPose;
                currentOdomPose = new Pose2d(
                    latestData.odom.x, 
                    latestData.odom.y, 
                    new Rotation2d(latestData.odom.w)
                );
                poseDeltas = new Pose2d(
                    currentOdomPose.getX() - prevOdomPose.getX(),
                    currentOdomPose.getY() - prevOdomPose.getY(),
                    currentOdomPose.getRotation().minus(prevOdomPose.getRotation())
                );

                if (latestData.vision.hasTargets) {
                    amcl.updateOdometry(poseDeltas.getX(), poseDeltas.getY(), poseDeltas.getRotation().getRadians());
                    amcl.tagScanning(latestData.vision.tagID, latestData.vision.distances, latestData.vision.campose);
                    publishEstimate(latestData.odom.id, amcl.getWeightedAverage().toPose2d(
                        Constants.FIELD_WIDTH / 2,
                        Constants.FIELD_HEIGHT / 2,
                        0
                    ));
                } else {
                    amcl.updateOdometry(poseDeltas.getX(), poseDeltas.getY(), poseDeltas.getRotation().getRadians());
                    publishEstimate(latestData.odom.id, amcl.getWeightedAverage().toPose2d(
                        Constants.FIELD_WIDTH / 2,
                        Constants.FIELD_HEIGHT / 2,
                        0
                    ));
                }
            } else {
                System.out.println("Connection dropped!");
                while (odomIDSub.get() == -1) {}
                System.out.println("Connection reestablished! Resetting AMCL...");
                amcl.resetMCL();
                System.out.println("Reinitialized AMCL!");
            }
        }
    }
}
