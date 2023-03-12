package frc.robot.controls;

import edu.wpi.first.wpilibj.Timer;

public class VelocityFromEncoder {
    double[] value_buffer;
    double[] timestamp_buffer;
    public VelocityFromEncoder(double initial_position, int buffersize){
        value_buffer = new double[buffersize];
        timestamp_buffer = new double[buffersize];
        
        for(int i = 0; i < buffersize; i++){
            value_buffer[i] = initial_position;
            timestamp_buffer[i] = getTime();
        }
    }
    public void update(double position){
        //Autohandle rotation lapping
        while(position - value_buffer[value_buffer.length - 1] > Math.PI) position -= 2 * Math.PI;
        while(position - value_buffer[value_buffer.length - 1] < -Math.PI) position += 2 * Math.PI;
        
        if(timestamp_buffer[timestamp_buffer.length - 1] != getTime()){
            for(int i = 0; i < value_buffer.length - 1; i++){
                value_buffer[i] = value_buffer[i + 1];
                timestamp_buffer[i] = timestamp_buffer[i + 1];
            }
            value_buffer[value_buffer.length - 1] = position;
            timestamp_buffer[timestamp_buffer.length - 1] = getTime();
        }
    }
    public double getRawVelocity(){
        if(timestamp_buffer[0] == timestamp_buffer[timestamp_buffer.length - 1]){
            return 0;
        }

        return (value_buffer[value_buffer.length - 1] - value_buffer[0])/(timestamp_buffer[timestamp_buffer.length - 1] - timestamp_buffer[0]);
    }
    public double getTime(){
        return Timer.getFPGATimestamp();
    }
}