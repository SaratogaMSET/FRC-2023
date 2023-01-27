package testclient;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FishClientNT {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable visionTable = inst.getTable("vision");

    private final NetworkTable odomTable = inst.getTable("odom");

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
    }

    public void start() {
        while (true) {
            
        }
    }
}
