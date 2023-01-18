package frc.robot.util.server;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FishServer implements Runnable {
    private Thread t;
    private int port;

    public FishServer(int port) {
        this.port = port;
    }

    public int getPort() {
        return port;
    }

    private void sendPoseData() {
        // TODO send drivetrain odometry data and botpose data from LL every time LL NetworkTables updates
        // ie. listen for updates from LL's NetworkTables, then send encoder and botpose data
    }

    @Override
    public void run() {
        System.out.println("Initializing webserver...");
        
        try (ServerSocket serverSocket = new ServerSocket(port)) {
            System.out.println("Server is listening on port " + port);
            SmartDashboard.putString("server test", "server good");

            while (true) {
                Socket socket = serverSocket.accept();

                System.out.println("New client connected. Client IP: " + socket.getRemoteSocketAddress());
                SmartDashboard.putString("client test", "client good");
                SmartDashboard.putString("client IP: ", socket.getRemoteSocketAddress().toString());

                OutputStream output = socket.getOutputStream();
                PrintWriter writer = new PrintWriter(output, true);

                writer.println("Current RIO FPGA timestamp: " + Timer.getFPGATimestamp());
            }
        } catch (IOException e) {
            System.out.println("Server exception: " + e);
            SmartDashboard.putString("test", "server bad");
            e.printStackTrace();
        }

        System.out.println("Stopping webserver.");
     }

    public void start() {
        System.out.println("Starting webserver on port " + port);
        if (t == null) {
            t = new Thread(this, "Server");
            t.start();
        }
    }
    
}
