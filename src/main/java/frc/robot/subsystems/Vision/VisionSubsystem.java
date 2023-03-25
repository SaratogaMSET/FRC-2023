package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable ll2 = NetworkTableInstance.getDefault().getTable("limelight-two");
    private NetworkTable ll3 = NetworkTableInstance.getDefault().getTable("limelight-three");

    private double d1;
    private double tx;

    private double a;  // x val 
    private double b;  // y val

    public VisionSubsystem() {}

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

    public long getTagId(){
        if (getPipeline() != 0) return -1;
        return getTable().getEntry("tid").getInteger(-1);
    }

    /* Retroreflective(NOT USEFUL) */

    public double getTX(){
        return getTable().getEntry("tx").getDouble(0.0);
    }

    private double getTY(){
        return getTable().getEntry("ty").getDouble(0.0);
    }

    private double getDistanceFromRetro(){
        double camHeight, camAngle;
        if (getPipeline() > 0){
            if (getTable().equals(ll3)){
                camHeight = VisionConstants.H1_LL3;
                camAngle = VisionConstants.A1_LL3;
            } else {
                camHeight = VisionConstants.H1_LL2;
                camAngle = VisionConstants.A1_LL2; 
            }
            if (getTY() < -3){
                return (Constants.VisionConstants.H2b - camHeight) / Math.tan(Math.toRadians(camAngle + getTY()));
            } 
            //SmartDashboard.putNumber("gET ME OUT", ((Constants.VisionConstants.H2a - camHeight) / Math.tan(Math.toRadians(camAngle + getTY()))));
            return (Constants.VisionConstants.H2a - camHeight) / Math.tan(Math.toRadians(camAngle + getTY()));
        } else {
            return 0.0;
        }
    }

    public double[] getOffsetTo2DOFBase(){       

        if (getTable().equals(ll3)){
            d1 = getDistanceFromRetro();
            tx = getTX();

            a = Math.sin(Math.toRadians(tx)) * d1;  // x val 
            b = Math.cos(Math.toRadians(tx)) * d1;  // y val

            double angle = Math.atan((a - Constants.VisionConstants.C2_LL3) / (b + Constants.VisionConstants.C1_LL3));
        double[] x = {a - Constants.VisionConstants.C2_LL3, b + Constants.VisionConstants.C1_LL3, Math.toDegrees(angle)};
            return x;
        } else {
            d1 = getDistanceFromRetro();
            tx = getTX();
            a = Math.sin(Math.toRadians(tx)) * d1;  // x val 
            b = Math.cos(Math.toRadians(tx)) * d1;  // y val

            double angle = Math.atan((a - Constants.VisionConstants.C2_LL3) / (b + Constants.VisionConstants.C1_LL3));
            
            // negative x -> arm is to the right. idk why man 
        double[] x = {a - Constants.VisionConstants.C2_LL2, b + Constants.VisionConstants.C1_LL2, Math.toDegrees(angle)};
            return x; 
        }
    }

    /**
     * *
     * @param pipelineNum the pipeline number to switch to. We might need one for the high tape, one for the mid tape and one for apriltag.
     */
    public void setPipeline(int pipelineNum){
        getTable().getEntry("pipeline").setNumber(pipelineNum);
    }
    /* End Retroreflective */

/* START OF ATREY'S APRILTAG CODE; USING OLD FUNCTIONS; RETURNS TX, TY, CAMERA-RELATIVE ANGLE TO APRILTAG */

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
        SmartDashboard.putNumberArray("Pose to target(arm base)", getOffsetTo2DOFBase()); //Ignore if not on retroreflective pipeline. 
        SmartDashboard.putNumber("distance to retro", getDistanceFromRetro());


        // SmartDashboard.putNumber("Raw Angle", getLatestResults().targetingResults.getBotPose2d().getRotation().getDegrees());


        // SmartDashboard.putNumber("LL to Mid (Horizontal)",getLimelightTx(1));
        // SmartDashboard.putNumber("LL to High (Horizontal)", getLimelightTx(2));
        // SmartDashboard.putNumber("LL to Mid (Vertical)",getLimelightTy(1));
        // SmartDashboard.putNumber("LL to High (Vertical)",getLimelightTy(2));
        // SmartDashboard.putNumber("LL to AprilTag",Math.hypot(getCamTranOld()[0], getCamTranOld()[2]));
        NetworkTableInstance.getDefault().flush();
        //meters to inches: 1m = 39.3701 in
        SmartDashboard.putNumber("LL to Mid (Horizontal, Inches)",39.3701*(getLimelightTx(1)));
        SmartDashboard.putNumber("LL to High (Horizontal, Inches)", 39.3701*(getLimelightTx(2)));
        SmartDashboard.putNumber("LL to Mid (Vertical, Inches)",39.3701*(getLimelightTy(1)));
        SmartDashboard.putNumber("LL to High (Vertical, Inches)",39.3701*(getLimelightTy(2)));
        SmartDashboard.putNumber("LL to AprilTag (Inches)",39.3701*(Math.hypot(getCamTranOld()[0], getCamTranOld()[2])));
        SmartDashboard.putNumber("getTX", getTX());
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void simulationPeriodic() {
        
    }
}