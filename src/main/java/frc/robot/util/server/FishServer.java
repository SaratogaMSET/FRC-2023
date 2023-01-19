package frc.robot.util.server;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
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
                ObjectOutputStream objOut = new ObjectOutputStream(output);
                // PrintWriter writer = new PrintWriter(objOut, true);

                InputStream input = socket.getInputStream();
                ObjectInputStream objIn = new ObjectInputStream(input);

                while (true) {
                    objOut.writeObject("Current RIO FPGA timestamp: " + Timer.getFPGATimestamp());
                    System.out.println(objIn.readObject());
                    Thread.sleep(500);
                }

                /* Thread t1 = new Thread() {
                    public void run() {
                        while (true) {
                            writer.println("Current RIO FPGA timestamp: " + Timer.getFPGATimestamp());
                            try {
                                this.wait();
                            } catch (InterruptedException e) {
                                // TODO Auto-generated catch block
                                e.printStackTrace();
                            }
                        }
                    }
                };
                t1.start();

                Thread t2 = new Thread() {
                    public void run() {
                        try {
                            while (true) {
                                String received = reader.readLine();
                                System.out.println("Received message from client: " + received);
                                notifyAll();
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                };
                t2.start(); */
            }
        } catch (IOException e) {
            System.out.println("Server exception: " + e);
            SmartDashboard.putString("test", "server bad");
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
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
