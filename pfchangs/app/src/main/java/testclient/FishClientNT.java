package testclient;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FishClientNT {
    private final NetworkTableInstance inst;
    private final NetworkTable rawTable;
    private final IntegerSubscriber measurementID;
    private final DoubleSubscriber fpgaTimeSub;
    private final DoubleSubscriber xSub;
    private final NetworkTable cookedTable;
    private final DoublePublisher xPub;

    public FishClientNT() {
        inst = NetworkTableInstance.getDefault();
        inst.startClient4("localizer");
        inst.setServer("localhost");

        rawTable = inst.getTable("rawRobotData");
        measurementID = rawTable.getIntegerTopic("id").subscribe(0);
        fpgaTimeSub = rawTable.getDoubleTopic("fpgaTime").subscribe(0d);
        xSub = rawTable.getDoubleTopic("x").subscribe(0d);

        cookedTable = inst.getTable("localized");
        xPub = cookedTable.getDoubleTopic("x").publish();
    }

    double x = 0;

    public void start() {
        System.out.println("Received values: " + measurementID.get() + ", " + fpgaTimeSub + ", " + xSub.get());
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
            return;
        }
        xPub.set(x++);
        inst.flush();
    }
}
