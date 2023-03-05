package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTable ll2 = NetworkTableInstance.getDefault().getTable("limelight-two");
    private NetworkTable ll3 = NetworkTableInstance.getDefault().getTable("limelight-three");

    public VisionSubsystem() {}

    private NetworkTable getTable(){
        if (ll3.getEntry("tv").getInteger(0) == 1){
            return ll3;
        } else {
            return ll2;
        }
    }

    public long getPipeline(){
        return getTable().getEntry("getpipe").getInteger(0);
    }

    public boolean hasTargets() {
        return table.getEntry("tv").getInteger(0) == 1;
    }

    /* Retroreflective(NOT USEFUL) */

    public double getTX(){
        return table.getEntry("tx").getDouble(0.0);
    }

    private double getTY(){
        return table.getEntry("ty").getDouble(0.0);
    }

    private double getDistanceFromRetro(){
        if (getTY() < -3){
            return 39.37 * (Constants.VisionConstants.H2b - Constants.VisionConstants.H1) / Math.tan(0.017453292519943295 * (Constants.VisionConstants.A1 + getTY()));
        } 
        return 39.37 * (Constants.VisionConstants.H2a - Constants.VisionConstants.H1) / Math.tan(0.017453292519943295 * (Constants.VisionConstants.A1 + getTY()));
    }

    public double[] getOffsetTo2DOFBase(){        

        double d1 = getDistanceFromRetro();
        double tx = getTX();

        double a = Math.sin(Math.toRadians(tx)) * d1;  // x val 
        double b = Math.cos(Math.toRadians(tx)) * d1;  // y val

        double angle = Math.atan((a + Constants.VisionConstants.C2) / (b + Constants.VisionConstants.C1));

        double[] x = {a + Constants.VisionConstants.C2, b + Constants.VisionConstants.C1, Math.toDegrees(angle)};

        return x;
    }

    /**
     * *
     * @param pipelineNum the pipeline number to switch to. We might need one for the high tape, one for the mid tape and one for apriltag.
     */
    public void switchPipeline(int pipelineNum){
        table.getEntry("pipeline").setNumber(pipelineNum);
    }
    /* End Retroreflective */

/* START OF ATREY'S APRILTAG CODE; USING OLD FUNCTIONS; RETURNS TX, TY, CAMERA-RELATIVE ANGLE TO APRILTAG */

    private double[] getCamTranOld() {
        return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
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