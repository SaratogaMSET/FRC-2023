package frc.robot.util.server;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.logging.LoggablePose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.wrappers.FilterEstimate;
import frc.robot.util.wrappers.SendableOdomMeasurement;
import frc.robot.util.wrappers.SendableVisionMeasurement;
import frc.robot.util.wrappers.SwerveOdomMeasurement;
import frc.robot.util.wrappers.TimestampedSwerveOdometry;
import frc.robot.util.wrappers.VisionMeasurement;

public class PoseEstimator {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;

    /**
     * Tracks measurements to associate between measurement and timestamp.
     */
    private int currentID = 0;
    /**
     * Converts ID to timestamp.
     */
    private HashMap<Integer, Double> idMap = new HashMap<>();
    /**
     * Associates timestamps with SwerveOdomMeasurements.
     */
    private ConcurrentSkipListMap<Double, SwerveOdomMeasurement> odomMap = new ConcurrentSkipListMap<>();
    /**
     * Interpolates timestamped estimates from swerve.
     */
    private TimeInterpolatableBuffer<Pose2d> poseMap = TimeInterpolatableBuffer.createBuffer(10);
    /**
     * Associates timestamps with odom and vision measurements.
     */
    private TreeMap<Double, Pair<SendableOdomMeasurement, SendableVisionMeasurement>> buffer = new TreeMap<>();

    /**
     * Stores raw (unfiltered) odometry data from swerve.
     */
    private TimestampedSwerveOdometry rawOdometry;
    /**
     * Stores latency-adjusted pose estimate calculated using raw swerve odom and particle filter estimates.
     */
    private TimestampedSwerveOdometry cookedOdometry;

    /**
     * Stores the most recently-received position estimate from the particle filter. Currently unused.
     */
    private FilterEstimate currentEstimate = new FilterEstimate(0, new Pose2d());

    /**
     * How old in seconds raw measurements need to be before they are published over NetworkTables.
     */
    private double timeThreshold = 0.2;

    private FishServerNT ntServer = new FishServerNT(this::computeEstimate);

    private LoggablePose logPose;
    private LoggablePose rawLogPose;

    public PoseEstimator(
        VisionSubsystem vision,
        DrivetrainSubsystem drivetrain,
        SwerveDriveKinematics kinematics, 
        Rotation2d initGyroAngle, 
        Pose2d prior
    ) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        rawOdometry = new TimestampedSwerveOdometry(kinematics, initGyroAngle, prior);
        cookedOdometry = new TimestampedSwerveOdometry(kinematics, initGyroAngle, prior);

        logPose = new LoggablePose("/Localizer/Pose", cookedOdometry.getPoseMeters(), true);
        rawLogPose = new LoggablePose("/Localizer/RawPose", rawOdometry.getPoseMeters(), true);
    }

    private void poseEstimatorPeriodic() {
        VisionMeasurement latestMeasurement = vision.getLatestMeasurement();
        SwerveOdomMeasurement odomMeasurement = new SwerveOdomMeasurement(
            drivetrain.getRotation2d(), 
            drivetrain.getModuleStates()
        );

        update(odomMeasurement, latestMeasurement);

        periodic();

        logPose.set(getPose());
        rawLogPose.set(rawOdometry.getPoseMeters());
    }

    private void update(SwerveOdomMeasurement odometryMeas, VisionMeasurement visionMeas) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometryMeas.getGyroAngle(), odometryMeas.getModuleStates());
        cookedOdometry.updateWithTime(currentTime, odometryMeas.getGyroAngle(), odometryMeas.getModuleStates());
        odomMap.put(currentTime, odometryMeas);

        poseMap.addSample(currentTime, rawOdometry.getPoseMeters());

        double visionTime = currentTime - (visionMeas.getLatency() / 1000);
        Pose2d interpolatedPose = poseMap.getSample(visionTime).get();

        SendableOdomMeasurement sendableOdom;
        SendableVisionMeasurement sendableVision = null;

        if (visionMeas.hasTargets()) {
            sendableVision = new SendableVisionMeasurement(
                0,
                true,
                visionMeas.getTagID(),
                visionMeas.getDistance(),
                new double[]{visionMeas.getCampose().getX(), visionMeas.getCampose().getY(), visionMeas.getCampose().getRotation().getRadians()}
            );
            sendableOdom = new SendableOdomMeasurement(0, interpolatedPose);
            buffer.put(visionTime, new Pair<SendableOdomMeasurement, SendableVisionMeasurement>(sendableOdom, sendableVision));
        } else {
            sendableOdom = new SendableOdomMeasurement(0, interpolatedPose);
            buffer.put(visionTime, new Pair<SendableOdomMeasurement, SendableVisionMeasurement>(sendableOdom, sendableVision));
        }
    }

    private void computeEstimate(FilterEstimate estimate) {
        currentEstimate = estimate;

        int estimateID = estimate.getID();
        double estimateTime = idMap.get(estimateID);

        TreeMap<Double, SwerveOdomMeasurement> tempOdomMap = new TreeMap<>(odomMap);

        Double lastKey = tempOdomMap.ceilingKey(estimateTime);

        cookedOdometry.resetPosition(estimate.getPose(), tempOdomMap.get(lastKey).getGyroAngle());

        Double prevTime = tempOdomMap.lowerKey(lastKey);
        if (prevTime != null) {
            while (lastKey != null) {
                SwerveOdomMeasurement lastMeasurement = tempOdomMap.get(lastKey);
                cookedOdometry.updateWithTime(prevTime, lastKey, lastMeasurement.getGyroAngle(), lastMeasurement.getModuleStates());
                prevTime = lastKey;
                tempOdomMap.remove(lastKey);
                lastKey = tempOdomMap.ceilingKey(lastKey);
            }
        }
    }

    public void reset(Pose2d pose, Rotation2d gyroAngle) {
        rawOdometry.resetPosition(pose, gyroAngle);
        cookedOdometry.resetPosition(pose, gyroAngle);
    }

    public FilterEstimate getCurrentEstimate() {
        return currentEstimate;
    }

    public Pose2d getPose() {
        return cookedOdometry.getPoseMeters();
    }

    private void periodic() {
        double currentTime = Timer.getFPGATimestamp();

        ArrayList<Double> keys = new ArrayList<>(buffer.keySet());
        for (double key : keys) {
            if (currentTime - key > timeThreshold) {
                currentID += 1;
                idMap.put(currentID, key);

                if (buffer.get(key).getSecond() != null) {
                    buffer.get(key).getFirst().setID(currentID);
                    buffer.get(key).getSecond().setMeasID(currentID);
                    ntServer.publish(
                        buffer.get(key).getFirst(), 
                        buffer.get(key).getSecond()
                    );
                } else {
                    buffer.get(key).getFirst().setID(currentID);
                    ntServer.publish(buffer.get(key).getFirst(), new SendableVisionMeasurement(currentID));
                }

                buffer.remove(key);
            }
        }
    }

    public void addOne() {
        ntServer.addOne();
    }

    public void start() {
        System.out.println("Starting FishServer.");
        new Thread("FishServer") {
            public void run() {
                while (true) {
                    poseEstimatorPeriodic();
                }
            }
        }.start();
        System.out.println("FishServer started.");
    }
}
