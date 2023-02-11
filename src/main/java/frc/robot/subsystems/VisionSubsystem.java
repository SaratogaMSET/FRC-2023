package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.wrappers.VisionMeasurement;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;

public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {}

    private double[] getCamTran() {
        double[] pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[10]);
        if (pose.length > 0) return pose;
        else return new double[6];
    }

    private double[] getBotPose() {
        double[] pose = getLatestResults().targetingResults.botpose;
        if (pose.length > 0) return pose;
        else return new double[6];
    }

    private int getTV() {
        return (int) table.getEntry("tv").getInteger(0);
    }

    private double getLatency() {
        return table.getEntry("tl").getDouble(-1) + 11;
    }

    /* Retroreflective(NOT USEFUL) */

    private double getTX(){
        return table.getEntry("tx").getDouble(0.0);
    }

    private double getTY(){
        return table.getEntry("ty").getDouble(0.0);
    }

    private double getDistanceFromRetro(){
        if (getTY() < 0){
            return (Constants.VisionConstants.H2b - Constants.VisionConstants.H1) / Math.tan(0.017453292519943295 * (Constants.VisionConstants.A1 + getTY()));
        } 
        return (Constants.VisionConstants.H2a - Constants.VisionConstants.H1) / Math.tan(0.017453292519943295 * (Constants.VisionConstants.A1 + getTY()));
    }

    private double[] getCornerData(){ // returns in ndc.    
        return table.getEntry("tcornxy").getDoubleArray(new double[12]);
    }

    // private double[] getOptimalTape(){ // returns tx, ty of optimal tape. closest, highest.
    //     for(int i = 0 ; i < 8; i += 2){
            
    //     }
    // }

    /**
     * *
     * @param pipelineNum the pipeline number to switch to. We might need one for the high tape, one for the mid tape and one for apriltag.
     */
    public void switchPipeline(int pipelineNum){
        table.getEntry("pipeline").setNumber(pipelineNum);
    }

    private int getTagID() {
        return (int) table.getEntry("tid").getInteger(-1);
    }


    private double[] getDistances() {
        double[] distances = new double[]{
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1
        };

        var tmp = new Pose2d();
        for (var r : getLatestResults().targetingResults.targets_Fiducials) {
            tmp = r.getRobotPose_TargetSpace2D();
            distances[(int) r.fiducialID] = Math.hypot(tmp.getX(), tmp.getY());
        }

        return distances;
    }

    /* Returns [x, y, theta] */
    public double[] getOffsetTo2DOFBase(){        

        double d1 = getDistanceFromRetro();
        double tx = getTX();

        double a = Math.sin(Math.toRadians(tx)) * d1;  // x val 
        double b = Math.cos(Math.toRadians(tx)) * d1;  // y val

        double angle = Math.atan((a + Constants.VisionConstants.C2) / (b + Constants.VisionConstants.C1));

        double[] x = {a + Constants.VisionConstants.C2, b + Constants.VisionConstants.C1, Math.toDegrees(angle)};

        return x;
    }

    public VisionMeasurement getLatestMeasurement() {
        double[] botpose = getBotPose();
        double[] campose = getCamTran();
        return new VisionMeasurement(
            getTV() != 0,
            getLatency(),
            getTagID(),
            new Pose2d(
                new Translation2d(botpose[0], botpose[2]),
                new Rotation2d(botpose[5])
            ),
            new Pose2d(
                new Translation2d(campose[0], campose[2]),
                new Rotation2d(campose[5])
            ),
            getDistances()
        );
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults("");
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Smart Targeting X", 100*Math.hypot(getCamTran()[0], getCamTran()[2])); //get X stuff for verification
        SmartDashboard.putNumberArray("Distances", getDistances());
        SmartDashboard.putNumberArray("Pose to target(arm base)", getOffsetTo2DOFBase()); //Ignore if not on retroreflective pipeline. 
        SmartDashboard.putNumber("distance to retro", getDistanceFromRetro());
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
