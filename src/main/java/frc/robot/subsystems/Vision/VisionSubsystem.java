package frc.robot.subsystems.Vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightResults;
import frc.robot.util.wrappers.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    /* Network tables is the method of communication between camera data  and us programmers. It's kinda like a dictionary
    */
    private NetworkTable ll2 = NetworkTableInstance.getDefault().getTable("limelight-two");
    private NetworkTable ll3 = NetworkTableInstance.getDefault().getTable("limelight-three");

    private double[] distances = new double[]{-1,-1,-1,-1,-1,-1,-1,-1};

    private double d1;
    private double tx;

    private double a;  // x val 
    private double b;  // y val


    public VisionSubsystem() {}


    /* gets the robot pose relative to the apriltag target. 
    Read docs: https://docs.limelightvision.io/en/latest/networktables_api.html
    Scroll to "Apriltags and 3D Data"
    */ 
    private Pose2d getCamPose2d() {
        double id = getTagID();
        for (var v : getLatestResults().targetingResults.targets_Fiducials) {
            if (v.fiducialID == id) return v.getRobotPose_TargetSpace2D();
        }

        return new Pose2d();
    }

    /* This is mostly for tuning the accuracy of the pipeline, the robot doesn't really use theses values well. Reports the 
    of the camera in meters from each apriltag based on id(array index 0 is apriltag 1) */ 
    private double[] getDistances() {
        Arrays.fill(distances, -1);

        for (var r : getLatestResults().targetingResults.targets_Fiducials) {
            var tmp = r.getRobotPose_TargetSpace();
            distances[(int) r.fiducialID - 1] = Math.hypot(tmp.getX(), tmp.getZ());
        }

        return distances;
    }

    /* Limelight helper implementation for server communication with deprecated mcl*/ 
    public VisionMeasurement getLatestMeasurement() {
        LimelightResults results = getLatestResults();
        return new VisionMeasurement(
            hasTargets(),
            results.targetingResults.latency_capture + 
                results.targetingResults.latency_pipeline + 
                results.targetingResults.latency_jsonParse,
            getTagID(),
            results.targetingResults.getBotPose2d(),
            getCamPose2d(),
            getDistances()
        );
    }

    // LL2 does not support the LimelightResults Helper functions. 
    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults("limelight-three");
    }

    // Rather than trying to fuse data together from cameras, we just decided to pick based on which camera has data
    // (Monkey solution) If both see data, ll3 is given priority. 
    public NetworkTable getTable(){
        if (ll3.getEntry("tv").getInteger(0) == 1){
            SmartDashboard.putNumber("check", 1);
            return ll3;
        } else {
            return ll2;
        }
    }

    public boolean isLL3(){
        if (getTable().equals(ll3)){
            return true; 
        } else {
            return false;
        }
    }

    /* Network Table Communications: https://docs.limelightvision.io/en/latest/networktables_api.html  */
  
    public long getPipeline(){
        return getTable().getEntry("getpipe").getInteger(0);
    }

    public boolean hasTargets() {
        return getTable().getEntry("tv").getInteger(0) == 1;
    }

    public int getTagID(){
        if (getPipeline() != 0) return -1;
        return (int) getTable().getEntry("tid").getInteger(-1);
    }

    public double getTX(){
        return getTable().getEntry("tx").getDouble(0.0);
    }

    private double getTY(){
        return getTable().getEntry("ty").getDouble(0.0);
    }

   /**
     * *
     * @param pipelineNum the pipeline number to switch to. We might need one for the high tape, one for the mid tape and one for apriltag.
     */
    public void setPipeline(int pipelineNum){
        getTable().getEntry("pipeline").setNumber(pipelineNum);
    }

    /* START OF ATREY'S APRILTAG CODE; USING OLD FUNCTIONS; RETURNS TX, TY, CAMERA-RELATIVE ANGLE TO APRILTAG */

    /* campose but using network tables */
    private double[] getCamTranOld() {
        return getTable().getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {

        // Logger.getInstance().recordOutput("Smart Targeting X", 100*Math.hypot(getCamTranOld()[0], getCamTranOld()[2])); //get X stuff for verification
        // SmartDashboard.putNumberArray("Botpose 2d", getLatestResults().targetingResults.botpose);
        // SmartDashboard.putNumberArray("Distances", getDistances());

        // SmartDashboard.putNumber("Raw Angle", getLatestResults().targetingResults.getBotPose2d().getRotation().getDegrees());

        // SmartDashboard.putNumber("LL to AprilTag",Math.hypot(getCamTranOld()[0], getCamTranOld()[2]));
        NetworkTableInstance.getDefault().flush();
        //meters to inches: 1m = 39.3701 in

        // SmartDashboard.putNumber("LL to AprilTag (Inches)",39.3701*(Math.hypot(getCamTranOld()[0], getCamTranOld()[2])));
        // SmartDashboard.putNumber("getTX", getTX());
    }

    @Override
    public void simulationPeriodic() {
        
    }
}