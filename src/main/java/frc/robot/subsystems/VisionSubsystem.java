package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.util.wrappers.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {
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

    public VisionSubsystem() {}

    private Pose2d getCamPose2d() {
        double id = getTagID();
        for (var v : getLatestResults().targetingResults.targets_Fiducials) {
            if (v.fiducialID == id) return v.getRobotPose_TargetSpace2D();
        }

        return new Pose2d();
    }

    private boolean hasTargets() {
        return table.getEntry("tv").getInteger(0) == 1;
    }

    private int getTagID() {
        return (int) table.getEntry("tid").getInteger(-1);
    }

    private double[] getDistances() {
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
            getDistances() // FIXME filtering distances
        );
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults("");
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
