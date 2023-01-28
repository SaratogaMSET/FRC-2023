package testclient;

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
    private final ParticleFilter filter = new ParticleFilter(0, null, 0, 0);

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

    public FishClientNT() {
        inst.startClient4("estimator");
        inst.setServer("localhost"); // "localhost" for simulation - FIXME change to actual robot
        // https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html

        System.out.println("Finshed client init.");

        System.out.println("Initializing particle filter.");

        System.out.println("Finished particle filter init.");
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
        }
    }
}
