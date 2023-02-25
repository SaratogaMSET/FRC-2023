package frc.robot.controls;

import java.io.FileReader;
import java.io.FileNotFoundException;
import org.json.simple.parser.*;
import org.json.simple.JSONObject;

public class TrajectoryParser {
    public class TrajectoryData {
        public double[][] ArmStates;
        public double[][] ArmControls;
        public double[] ArmTimestamps;
        
        private double startTime = 0;
        private double endTime = 0;
        public TrajectoryData(double[][] states, double[][] controls, double[] timestamps){
            this.ArmStates = states;
            this.ArmControls = controls;
            this.ArmTimestamps = timestamps;
            this.startTime = timestamps[0];
            this.endTime = timestamps[timestamps.length - 1];
        }
        public int timestepIndexFloor(double time){
            if(time > this.endTime){
                return this.ArmTimestamps.length - 1;
            }else if(time < this.startTime){
                return -1;
            }

            int currentIndex = 0;
            while(this.ArmTimestamps[currentIndex] < time) currentIndex ++;

            return currentIndex - 1;
        }
        public double[] stateAtTime(double time){
            int index = timestepIndexFloor(time);

            if(index == ArmTimestamps.length - 1){
                return ArmStates[index];
            }else if(index == -1){
                return ArmStates[0];
            }
            
            int num_states = 4;
            double indexPercentage = (time - ArmTimestamps[index])/(ArmTimestamps[index + 1] - ArmTimestamps[index]);
            double[] lerpedState = new double[num_states];
            for(int i = 0; i < num_states; i++){
                lerpedState[i] = ArmStates[index][i] + indexPercentage * (ArmStates[index + 1][i] - ArmStates[index][i]);
            }

            return lerpedState;
        }
        public double[] controlAtTime(double time){
            int index = timestepIndexFloor(time);

            if(index == ArmTimestamps.length - 1){
                return ArmControls[index];
            }else if(index == -1){
                return ArmControls[0];
            }
            
            int num_controls = 2;
            double indexPercentage = (time - ArmTimestamps[index])/(ArmTimestamps[index + 1] - ArmTimestamps[index]);
            double[] lerpedControl = new double[num_controls];
            for(int i = 0; i < num_controls; i++){
                lerpedControl[i] = ArmControls[index][i] + indexPercentage * (ArmControls[index + 1][i] - ArmControls[index][i]);
            }

            return lerpedControl;
        }
    }

    public TrajectoryData parse(String path_name) throws Exception{
        JSONObject file = (JSONObject) new JSONParser().parse(new FileReader("TrajectoryData/" + path_name + ".json"));
        
        
        double[][] state_data = (double[][]) file.get("state_data");
        double[][] control_data = (double[][]) file.get("control_data");
        double[] time_data = (double[]) file.get("time_data");
        
        return new TrajectoryData(state_data, control_data, time_data);
    }
}
