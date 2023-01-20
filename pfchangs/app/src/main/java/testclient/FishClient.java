package testclient;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;

public class FishClient {
    private final String hostname;
    private final int port;
    private Socket socket;
    private boolean reconnect = true;
    private final int heartbeatDelay = 500; // Send test signal every 500ms
    private final Thread heartbeat;

    private Object receivedData;

    public FishClient(final String hostname, final int port) {
        this.hostname = hostname;
        this.port = port;
        
        heartbeat = new Thread() {
            public void run() {
                while (reconnect) {
                    try {
                        socket.getOutputStream().write(649);
                        Thread.sleep(heartbeatDelay);
                    } catch (InterruptedException e) {
                        // reconnect = false;
                    } catch (IOException e) {
                        System.out.println("Server at " + hostname + " on port " + port + " is offline");
                        connect(hostname, port);
                    } catch (Exception e) {
                        System.out.println("Error with server at " + hostname + " on port " + port + ".");
                        connect(hostname, port);
                    }
                }
            }
        };
    }

    public void start() {
        connect(hostname, port);
        heartbeat.start();
    }

    private void connect(String hostname, int port) {
        try {
            socket = new Socket(hostname, port);

            testDoAThing();
        } catch (UnknownHostException e) {
            System.out.println("Server not found: " + e.getMessage());
        } catch (IOException e) {
            System.out.println("I/O Error: " + e.getMessage());
        }
    }

    private void testDoAThing() {
        try {
            OutputStream output = socket.getOutputStream();
            ObjectOutputStream objOut = new ObjectOutputStream(output);

            InputStream input = socket.getInputStream();
            ObjectInputStream objIn = new ObjectInputStream(input);

            while (true) {
                receivedData = objIn.readObject();
                System.out.println(receivedData);
                objOut.writeObject("Received data: " + receivedData);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        } catch (IOException e) {
            System.out.println("I/O Error: " + e.getMessage());
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void shutdown() {
        reconnect = false;
    }
}
