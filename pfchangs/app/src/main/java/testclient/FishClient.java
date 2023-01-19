package testclient;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
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
            /* InputStream stream = socket.getInputStream();
            BufferedReader reader = new BufferedReader(new InputStreamReader(stream));

            OutputStream oStream = socket.getOutputStream();
            PrintWriter writer = new PrintWriter(oStream, true); */

            OutputStream output = socket.getOutputStream();
            ObjectOutputStream objOut = new ObjectOutputStream(output);

            InputStream input = socket.getInputStream();
            ObjectInputStream objIn = new ObjectInputStream(input);

            while (true) {
                var s = objIn.readObject();
                System.out.println(s);
                objOut.writeObject("Received data: " + s);
            }

            /* while (true) {
                String fpgaTime = reader.readLine();
                System.out.println("Client checkpoint 1");
                System.out.println(fpgaTime);
                System.out.println("Client checkpoint 2");
                Thread.sleep(500);
                writer.println("Received fpgaTime " + fpgaTime + " from RIO.");
            } */
        } catch (IOException e) {
            System.out.println("I/O Error: " + e.getMessage());
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }/* catch (InterruptedException e) {
            e.printStackTrace();
        } */
    }

    public void shutdown() {
        reconnect = false;
    }
}
