package testclient;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;

public class FishClient {
    private Socket socket;
    private boolean reconnect = true;
    private final int heartbeatDelay = 500; // Send test signal every 500ms
    private final Thread heartbeat;

    public FishClient(final String hostname, final int port) {
        connect(hostname, port);

        heartbeat = new Thread() {
            public void run() {
                while (reconnect) {
                    try {
                        socket.getOutputStream().write(649);
                        sleep(heartbeatDelay);
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
            InputStream stream = socket.getInputStream();
            BufferedReader reader = new BufferedReader(new InputStreamReader(stream));

            String fpgaTime = reader.readLine();
            System.out.println(fpgaTime);
        } catch (IOException e) {
            System.out.println("I/O Error: " + e.getMessage());
        }
    }

    public void shutdown() {
        reconnect = false;
    }
}
