package frc.robot.util.server;

import java.util.HashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.wrappers.FilterEstimate;
import frc.robot.util.wrappers.SendableOdomMeasurement;
import frc.robot.util.wrappers.SendableVisionMeasurement;
import frc.robot.util.wrappers.SwerveOdomMeasurement;
import frc.robot.util.wrappers.TimestampedSwerveOdometry;
import frc.robot.util.wrappers.VisionMeasurement;

public class PoseEstimator implements Runnable {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;

    private int currentID = 0;
    private HashMap<Integer, Double> idMap = new HashMap<>(); // Convert measurement IDs to timestamps
    private ConcurrentSkipListMap<Double, SwerveOdomMeasurement> odomMap = new ConcurrentSkipListMap<>(); // FIXME potentially: see 604's code
    private TimeInterpolatableBuffer<Pose2d> poseMap = TimeInterpolatableBuffer.createBuffer(10);
    private TreeMap<Double, Pair<SendableOdomMeasurement, SendableVisionMeasurement>> buffer = new TreeMap<>();

    private TimestampedSwerveOdometry rawOdometry;
    private TimestampedSwerveOdometry cookedOdometry;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable visionTable = inst.getTable("vision");
    private NetworkTable odomTable = inst.getTable("odom");
    private NetworkTable estimateTable = inst.getTable("estimated");

    private FilterEstimate currentEstimate = new FilterEstimate(0, new Pose2d());

    private double timeThreshold = 0.2;

    public PoseEstimator(
        VisionSubsystem vision,
        DrivetrainSubsystem drivetrain,
        String name, 
        SwerveDriveKinematics kinematics, 
        Rotation2d initGyroAngle, 
        SwerveModulePosition[] modulePositions, 
        Pose2d prior
    ) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        rawOdometry = new TimestampedSwerveOdometry(kinematics, initGyroAngle, prior);
        cookedOdometry = new TimestampedSwerveOdometry(kinematics, initGyroAngle, prior);
    }

    private void poseEstimatorPeriodic() {
        VisionMeasurement latestMeasurement = vision.getLatestMeasurement();
        SwerveOdomMeasurement odomMeasurement = new SwerveOdomMeasurement( // potential FIXME pending DT finalization
            drivetrain.getRotation2d(), 
            drivetrain.getCurrentStates() // TODO check if the order is same as 604: FL --> FR --> BL --> BR
        );

        if (latestMeasurement.hasTargets()) {
            update(odomMeasurement, latestMeasurement);
        } else {
            update(odomMeasurement);
        }

        periodic();
    }

    private void update(SwerveOdomMeasurement odometry, VisionMeasurement vision) {
        double currentTime = Timer.getFPGATimestamp();
    }

    private void update(SwerveOdomMeasurement odometry) {

    }

    private void periodic() {

    }

    @Override
    public void run() {
        while (true) {
            poseEstimatorPeriodic();
        }
    }
}
