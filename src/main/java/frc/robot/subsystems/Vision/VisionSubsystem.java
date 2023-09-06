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

    private NetworkTable ll2 = NetworkTableInstance.getDefault().getTable("limelight-two");
    private NetworkTable ll3 = NetworkTableInstance.getDefault().getTable("limelight-three");

    private double[] distances = new double[]{-1,-1,-1,-1,-1,-1,-1,-1};

    private double d1;
    private double tx;

    private double a;  // x val 
    private double b;  // y val


    public VisionSubsystem() {}


    /* gets the robot pose relative to the apriltag target. Read docs */ 
    private Pose2d getCamPose2d() {
        double id = getTagID();
        for (var v : getLatestResults().targetingResults.targets_Fiducials) {
            if (v.fiducialID == id) return v.getRobotPose_TargetSpace2D();
        }

        return new Pose2d();
    }

    /* This is mostly for tuning the accuracy of the pipeline, the robot doesn't really use theses values well  */ 
    private double[] getDistances() {
        Arrays.fill(distances, -1);

        for (var r : getLatestResults().targetingResults.targets_Fiducials) {
            var tmp = r.getRobotPose_TargetSpace();
            distances[(int) r.fiducialID - 1] = Math.hypot(tmp.getX(), tmp.getZ());
        }

        return distances;
    }

    /* Limelight helper implementation for deprecated mcl */ 
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

    public double getLimelightTx(int topOrMid){
        double apriltagDistance = Math.hypot(getCamTranOld()[0], getCamTranOld()[2]);

        if(topOrMid==1){
            return apriltagDistance+Constants.Vision.apriltagToMidHorizontal;
        }
        else if(topOrMid==2){
            return apriltagDistance+Constants.Vision.apriltagToHighHorizontal;
        }
        else{
            return -1;
        }
    }

    public double getLimelightTy(int topOrMid){
        if(topOrMid==1){
            return 0.6477;
        }
        else if(topOrMid==2){
            return 0.9652;
        }
        else{
            return -1;
        }
    }

    @Override
    public void periodic() {

        // Logger.getInstance().recordOutput("Smart Targeting X", 100*Math.hypot(getCamTranOld()[0], getCamTranOld()[2])); //get X stuff for verification
        // SmartDashboard.putNumberArray("Botpose 2d", getLatestResults().targetingResults.botpose);
        // SmartDashboard.putNumberArray("Distances", getDistances());
        // SmartDashboard.putNumberArray("Pose to target(arm base)", getOffsetTo2DOFBase()); //Ignore if not on retroreflective pipeline. 
        // SmartDashboard.putNumber("distance to retro", getDistanceFromRetro());


        // SmartDashboard.putNumber("Raw Angle", getLatestResults().targetingResults.getBotPose2d().getRotation().getDegrees());


        // SmartDashboard.putNumber("LL to Mid (Horizontal)",getLimelightTx(1));
        // SmartDashboard.putNumber("LL to High (Horizontal)", getLimelightTx(2));
        // SmartDashboard.putNumber("LL to Mid (Vertical)",getLimelightTy(1));
        // SmartDashboard.putNumber("LL to High (Vertical)",getLimelightTy(2));
        // SmartDashboard.putNumber("LL to AprilTag",Math.hypot(getCamTranOld()[0], getCamTranOld()[2]));
        NetworkTableInstance.getDefault().flush();
        //meters to inches: 1m = 39.3701 in
        // SmartDashboard.putNumber("LL to Mid (Horizontal, Inches)",39.3701*(getLimelightTx(1)));
        // SmartDashboard.putNumber("LL to High (Horizontal, Inches)", 39.3701*(getLimelightTx(2)));
        // SmartDashboard.putNumber("LL to Mid (Vertical, Inches)",39.3701*(getLimelightTy(1)));
        // SmartDashboard.putNumber("LL to High (Vertical, Inches)",39.3701*(getLimelightTy(2)));
        // SmartDashboard.putNumber("LL to AprilTag (Inches)",39.3701*(Math.hypot(getCamTranOld()[0], getCamTranOld()[2])));
        // SmartDashboard.putNumber("getTX", getTX());
        // NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void simulationPeriodic() {
        
    }
}