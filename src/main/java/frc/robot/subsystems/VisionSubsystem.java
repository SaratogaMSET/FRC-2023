package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.util.wrappers.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {
    // private final double filterTimeConstant = 0.02;
    // private final double filterPeriod = 0.005;
    private final HashMap<Integer, LinearFilter> linearFilters = new HashMap<>(){{
        /* put(0, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(1, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(2, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(3, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(4, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(5, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(6, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod));
        put(7, LinearFilter.singlePoleIIR(filterTimeConstant, filterPeriod)); */
        put(0, LinearFilter.movingAverage(10));
        put(1, LinearFilter.movingAverage(10));
        put(2, LinearFilter.movingAverage(10));
        put(3, LinearFilter.movingAverage(10));
        put(4, LinearFilter.movingAverage(10));
        put(5, LinearFilter.movingAverage(10));
        put(6, LinearFilter.movingAverage(10));
        put(7, LinearFilter.movingAverage(10));
    }};

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double[] rawDistances = new double[]{
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1
    };
    private double[] distances = new double[]{
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1
    };

    public VisionSubsystem() {}

    private Pose2d getCamPose2d() {
        double id = getTagID();
        for (var v : getLatestResults().targetingResults.targets_Fiducials) {
            if (v.fiducialID == id) return v.getRobotPose_TargetSpace2D();
        }

        return new Pose2d();
    }

    private double[] getCamTran() {
        double id = getTagID();
        for (var v : getLatestResults().targetingResults.targets_Fiducials) {
            if (v.fiducialID == id) return new double[]{
                v.getRobotPose_TargetSpace2D().getX(),
                v.getRobotPose_TargetSpace2D().getY(),
                v.getRobotPose_TargetSpace2D().getRotation().getDegrees()
            };
        }

        double[] pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[10]);
        if (pose.length > 0) return pose;
        else return new double[6];
    }

    private boolean hasTargets() {
        return table.getEntry("tv").getInteger(0) == 1;
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

    private double[] getRawDistances() {
        rawDistances = new double[]{
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1
        };

        for (var r : getLatestResults().targetingResults.targets_Fiducials) {
            var tmp = r.getRobotPose_TargetSpace();
            rawDistances[(int) r.fiducialID - 1] = Math.hypot(tmp.getX(), tmp.getZ());
        }

        return rawDistances;
    }

    private double[] getDistances() {
        getRawDistances();
        for (int i = 0; i < rawDistances.length; ++i) {
            /* if (rawDistances[i] > 0)  */distances[i] = linearFilters.get(i).calculate(distances[i]);
            /* else {
                distances[i] = -1;
                linearFilters.get(i).calculate(rawDistances[i]);
            } */
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
        LimelightResults results = getLatestResults();
        return new VisionMeasurement(
            hasTargets(),
            results.targetingResults.latency_capture + 
                results.targetingResults.latency_pipeline + 
                results.targetingResults.latency_jsonParse,
            getTagID(),
            results.targetingResults.getBotPose2d(),
            getCamPose2d(),
            getRawDistances() // FIXME distance filter
        );
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults("");
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Smart Targeting X", 100*Math.hypot(getCamTran()[0], getCamTran()[2])); //get X stuff for verification
        SmartDashboard.putNumberArray("Botpose 2d", getLatestResults().targetingResults.botpose);
        SmartDashboard.putNumber("Botpose rotation", getLatestResults().targetingResults.getBotPose2d().getRotation().getDegrees());
        SmartDashboard.putNumberArray("Distances", getDistances());
        SmartDashboard.putNumberArray("Pose to target(arm base)", getOffsetTo2DOFBase()); //Ignore if not on retroreflective pipeline. 
        SmartDashboard.putNumber("distance to retro", getDistanceFromRetro());
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
