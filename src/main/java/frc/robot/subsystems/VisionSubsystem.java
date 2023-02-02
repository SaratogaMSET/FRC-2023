package frc.robot.subsystems;

import org.apache.commons.collections4.map.PassiveExpiringMap.ConstantTimeToLiveExpirationPolicy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.wrappers.VisionMeasurement;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {}

    private double[] getCamTran() {
        return table.getEntry("campose").getDoubleArray(new double[10]);
    }

    private double[] getBotPose() {
        double[] pose = table.getEntry("botpose").getDoubleArray(new double[10]);
        if (pose.length > 0) return pose;
        else return new double[8];
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
        return (Constants.VisionConstants.H2 - Constants.VisionConstants.H1) / Math.tan(Math.toRadians(Constants.VisionConstants.A1 + getTY()));
    }

    private int getTagID() {
        return (int) table.getEntry("tid").getInteger(-1);
    }


    private double[] getDistances() {
        // TODO check if we can get number of tags in view, if we can, switch between this method and botpose-->all 8 tag distances
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
        int tagID = getTagID();
        if (tagID > 0) {
            // TODO check distance calc (are we using the right values from camtran, should we even be using camtran, etc.)
            distances[tagID -1 ] = Math.hypot(getCamTran()[0], getCamTran()[2]);
            // distances[tagID - 1] = Math.sqrt(Math.pow(getCamTran()[0], 2) + Math.pow(getCamTran()[2], 2));
        }

        return distances;
    }

    /* Returns [x, y, theta] */
    public double[] getOffsetTo2DOFBase(){        

        double d1 = getDistanceFromRetro();
        double tx = getTX();

        double a = Math.sin(tx) * d1;  // x val 
        double b = Math.cos(tx) * d1;  // y val

        double angle = Math.atan((a + Constants.VisionConstants.C2) / (b + Constants.VisionConstants.C1));

        double[] x = {a + Constants.VisionConstants.C2, b + Constants.VisionConstants.C1, angle};

        return x;
    }

    public VisionMeasurement getLatestMeasurement() {
        double[] botpose = getBotPose();
        return new VisionMeasurement(
            getTV() != 0,
            getLatency(),
            getTagID(),
            new Pose2d(
                new Translation2d(botpose[0], botpose[2]),
                new Rotation2d(botpose[5])
            ),
            getDistances()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Distances", getDistances());
        SmartDashboard.putNumberArray("Pose to target(arm base)", getOffsetTo2DOFBase());
    }

    @Override
    public void simulationPeriodic() {
        
    }
}
