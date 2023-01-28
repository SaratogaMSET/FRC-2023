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
import testclient.filter.ParticleFilter;
import testclient.wrappers.RobotData;

public class FishClientNT {
    private final ParticleFilter filter = new ParticleFilter(
        Constants.FilterConstants.NUM_PARTICLES, 
        Constants.VisionConstants.Field.TAGS, 
        Constants.VisionConstants.Field.FIELD_WIDTH, 
        Constants.VisionConstants.Field.FIELD_HEIGHT
    );

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable visionTable = inst.getTable("vision");
    private final IntegerSubscriber visionIDSub = visionTable.getIntegerTopic("id").subscribe(-1);
    private final BooleanSubscriber hasTargetsSub = visionTable.getBooleanTopic("hasTargets").subscribe(false);
    private final IntegerSubscriber tagIDSub = visionTable.getIntegerTopic("tagID").subscribe(-1);
    private final DoubleArraySubscriber distanceSub = visionTable.getDoubleArrayTopic("distances").subscribe(new double[]{-1, -1, -1, -1, -1, -1, -1, -1});

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

    private Pose2d prevOdomPose = new Pose2d();
    private Pose2d currentOdomPose = new Pose2d();
    private Pose2d poseDeltas = new Pose2d();

    public FishClientNT() {
        inst.startClient4("estimator");
        inst.setServer("localhost"); // "localhost" for simulation - FIXME change to actual robot
        // https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html

        System.out.println("Finshed client init.");

        System.out.println("Starting filter init.");
        filter.setNoise(
            Constants.FilterConstants.FNOISE, 
            Constants.FilterConstants.TNOISE, 
            Constants.FilterConstants.SNOISE
        );
    }

    private RobotData readNTData() {
        return new RobotData(
            (int) visionIDSub.get(), 
            hasTargetsSub.get(), 
            (int) tagIDSub.get(), 
            distanceSub.get(), 
            (int) odomIDSub.get(),
            odomXSub.get(),
            odomYSub.get(),
            odomWSub.get()
        );
    }

    public void start() {
        while (true) {
            RobotData latestData = readNTData();
            prevOdomPose = currentOdomPose; // TODO we can probably optimize this
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
                filter.move(poseDeltas.getX(), poseDeltas.getY(), poseDeltas.getRotation().getRadians());
                filter.resample(latestData.vision.distances);
            } else {
                filter.move(poseDeltas.getX(), poseDeltas.getY(), poseDeltas.getRotation().getRadians());
            }
        }
    }
}
