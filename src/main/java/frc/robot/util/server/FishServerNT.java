package frc.robot.util.server;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class FishServerNT implements Runnable {
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final IntegerPublisher measurementID;
    private final DoublePublisher fpgaTimePub;
    private final DoublePublisher xPub;
    private Thread t;

    public FishServerNT() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("rawRobotData");
        measurementID = table.getIntegerTopic("id").publish();
        fpgaTimePub = table.getDoubleTopic("fpgaTime").publish();
        xPub = table.getDoubleTopic("x").publish();
    }

    int id = 0;
    double x = 0;

    @Override
    public void run() {
        while (true) {
            measurementID.set(id);
            fpgaTimePub.set(Timer.getFPGATimestamp());
            xPub.set(x++);
            inst.flush();
        }

    }

    public void start() {
        System.out.println("Starting FishServer.");
        if (t == null) {
            t = new Thread(this, "server");
            t.start();
        }
    }
}
