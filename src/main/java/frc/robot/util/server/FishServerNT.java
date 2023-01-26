package frc.robot.util.server;

public class FishServerNT implements Runnable {    
    private Thread t;

    public FishServerNT() {}

    @Override
    public void run() {
        while (true) {
            
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
