package testclient;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FishClientNT {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();;
    private final NetworkTable visionTable = inst.getTable("vision");
    private final NetworkTable odomTable = inst.getTable("odom");
    private final NetworkTable estimateTable = inst.getTable("estimated");

    public FishClientNT() {
        inst.startClient4("estimator");
        inst.setServer("localhost"); // "localhost" for simulation - FIXME change to actual robot
        // https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html

        System.out.println("Finshed client init.");
    }

    double x = 0;

    public void start() {
        while (true) {
            
        }
    }
}
