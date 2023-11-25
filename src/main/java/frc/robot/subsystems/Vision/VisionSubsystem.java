package frc.robot.subsystems.Vision;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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

    // Optional denotes that it can be null! Maps the limelight field coordinates to the robot field coordinates(they have different origin points)
    public Optional<Pose2d> getBotPose2d() {
        double[] arr = getTable().getEntry("botpose").getDoubleArray(new double[8]);
        
        if (getTable().getEntry("tv").getDouble(0) == 0) return Optional.empty();
        else return Optional.of(new Pose2d(new Translation2d(arr[0] + (8.24), arr[1] + 4.065), Rotation2d.fromDegrees(arr[5])));
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

    // Calculates timestamp for pose filtering according to limelight docs(FPGA - tl - cl)
    public double getTimestamp(){

        double timestamp = Timer.getFPGATimestamp() - (getTable().getEntry("tl").getDouble(1) / 1000  - getTable().getEntry("cv").getDouble(1)) / 1000;
        return timestamp;
    }

    // Returns true if the robot can see a target
    public boolean hasTargets() {
        return getTable().getEntry("tv").getInteger(0) == 1;
    }

    // Returns id of most visible tag. 
    public int getTagID(){
        if (getPipeline() != 0) return -1;
        return (int) getTable().getEntry("tid").getInteger(-1);
    }

    // Read docs. (target in degrees from center point of camera vision)
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
    // o7 

    /* campose but using network tables */
    private double[] getCamTranOld() {
        return getTable().getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {

        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void simulationPeriodic() {
        
    }
}