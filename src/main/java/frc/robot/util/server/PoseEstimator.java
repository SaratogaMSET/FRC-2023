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

    private int currentID = 0;
    private HashMap<Integer, Double> idMap = new HashMap<>(); // Convert measurement IDs to timestamps
    private ConcurrentSkipListMap<Double, SwerveOdomMeasurement> odomMap = new ConcurrentSkipListMap<>(); // FIXME potentially: see 604's code
    private TimeInterpolatableBuffer<Pose2d> poseMap = TimeInterpolatableBuffer.createBuffer(10);
    private TreeMap<Double, Pair<SendableOdomMeasurement, SendableVisionMeasurement>> buffer = new TreeMap<>();

    private TimestampedSwerveOdometry rawOdometry;
    private TimestampedSwerveOdometry cookedOdometry;

    private FilterEstimate currentEstimate = new FilterEstimate(0, new Pose2d());

    private double timeThreshold = 0.2;

    private FishServerNT ntServer = new FishServerNT(this::computeEstimate);

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
    }

    private void poseEstimatorPeriodic() {
        VisionMeasurement latestMeasurement = vision.getLatestMeasurement();
        SwerveOdomMeasurement odomMeasurement = new SwerveOdomMeasurement( // potential FIXME pending DT finalization
            drivetrain.getRotation2d(), 
            drivetrain.getModuleStates() // TODO check if the order is same as 604: FL --> FR --> BL --> BR
        );

        if (latestMeasurement.hasTargets()) {
            update(odomMeasurement, latestMeasurement);
        } else {
            update(odomMeasurement);
        }

        periodic();
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
            sendableVision = new SendableVisionMeasurement(0, true, visionMeas.getTagID(), visionMeas.getDistance());
            sendableOdom = new SendableOdomMeasurement(0, interpolatedPose);
            buffer.put(visionTime, new Pair<SendableOdomMeasurement, SendableVisionMeasurement>(sendableOdom, sendableVision));
        } else {
            sendableOdom = new SendableOdomMeasurement(0, interpolatedPose);
            buffer.put(visionTime, new Pair<SendableOdomMeasurement, SendableVisionMeasurement>(sendableOdom, sendableVision));
        }
    }

    private void update(SwerveOdomMeasurement odometryMeas) {
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometryMeas.getGyroAngle(), odometryMeas.getModuleStates());
        cookedOdometry.updateWithTime(currentTime, odometryMeas.getGyroAngle(), odometryMeas.getModuleStates());
        odomMap.put(currentTime, odometryMeas);

        poseMap.addSample(currentTime, rawOdometry.getPoseMeters());

        buffer.put(currentTime, new Pair<SendableOdomMeasurement, SendableVisionMeasurement>(
            new SendableOdomMeasurement(0, rawOdometry.getPoseMeters()),
            null
        ));
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
                    ntServer.publishAll(
                        buffer.get(key).getFirst(), 
                        buffer.get(key).getSecond()
                    );
                } else {
                    buffer.get(key).getFirst().setID(currentID);
                    ntServer.publishOdometry(buffer.get(key).getFirst());
                }

                buffer.remove(key);
            }
        }
    }

    public void start() {
        System.out.println("Starting FishServer.");
        new Thread() {
            public void run() {
                while (true) {
                    poseEstimatorPeriodic();
                }
            }
        }.start();
        System.out.println("FishServer started.");
    }
}
