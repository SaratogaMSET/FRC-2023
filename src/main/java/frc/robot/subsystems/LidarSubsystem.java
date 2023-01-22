package frc.robot.subsystems;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;

public class LidarSubsystem  {
	private I2C i2c;
	private byte[] distance;
	private java.util.Timer updater;
	
	private final int LIDAR_ADDR = 0x52; // laser address
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
    public LidarSubsystem(Port port) {    	
    	i2c = new I2C(port, LIDAR_ADDR);
		distance = new byte[2];
		updater = new java.util.Timer();
    }
    
    public void initDefaultCommand() {
    }
    
    // Distance in cm
 	public double getDistance() {
 		return (double)(Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]) / 2.54);
 	}
  
 	// gets double of getDistance value
 	public double pidGet() {
 		return getDistance(); // returns distance
 	}
 	
 	// Start 10Hz polling
 	public void start() {
 		updater.scheduleAtFixedRate(new LIDARUpdater(), 0, 100); // schedules updater at 100ms
 	}
 	
 	// Start polling for period in milliseconds
 	public void start(int period) {
 		updater.scheduleAtFixedRate(new LIDARUpdater(), 0, period); // schedules updater for inputed period
 	}
 	
 	// stops the laser
 	public void stop() {
 		updater.cancel();
 		updater = new java.util.Timer();
 	}
 	
 	// Update distance variable
 	public void update() {
 		i2c.write(LIDAR_CONFIG_REGISTER, 0x04);
 		Timer.delay(0.04); // Delay for measurement to be taken
 		i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement
 		Timer.delay(0.005); // Delay to prevent over polling
 	}
 	
 	public void write(){
 		i2c.write(LIDAR_CONFIG_REGISTER, 0x04);
 	}
 	
 	// reads distance
 	public void read(){
 		i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance);
 	}
 	
 	// Timer task to keep distance updated
 	private class LIDARUpdater extends TimerTask {
 		public void run() {
 			while(true) {
 				update();
 				try {
 					Thread.sleep(10);
 				} catch (InterruptedException e) {
 					e.printStackTrace();
 				}
 			}
 		}
 	}
 	
    protected double returnPIDInput() {
    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {
    }
}