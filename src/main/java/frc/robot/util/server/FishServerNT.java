package frc.robot.util.server;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.wrappers.FilterEstimate;
import frc.robot.util.wrappers.SendableOdomMeasurement;
import frc.robot.util.wrappers.SendableVisionMeasurement;

public class FishServerNT {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable visionTable = inst.getTable("vision");
    private final IntegerPublisher visionIDPub = visionTable.getIntegerTopic("id").publish();
    private final BooleanPublisher hasTargetsPub = visionTable.getBooleanTopic("hasTargets").publish();
    private final IntegerPublisher tagIDPub = visionTable.getIntegerTopic("tagID").publish();
    private final DoubleArrayPublisher distancePub = visionTable.getDoubleArrayTopic("distances").publish();
    private final DoubleArrayPublisher distanceXPub = visionTable.getDoubleArrayTopic("distanceX").publish();
    private final DoubleArrayPublisher distanceYPub = visionTable.getDoubleArrayTopic("distanceY").publish();
    private final DoubleArrayPublisher camposePub = visionTable.getDoubleArrayTopic("campose").publish();

    private final NetworkTable odomTable = inst.getTable("odom");
    private final IntegerPublisher odomIDPub = odomTable.getIntegerTopic("id").publish();
    private final DoublePublisher odomXPub = odomTable.getDoubleTopic("x").publish();
    private final DoublePublisher odomYPub = odomTable.getDoubleTopic("y").publish();
    private final DoublePublisher odomWPub = odomTable.getDoubleTopic("w").publish();

    private final NetworkTable estimateTable = inst.getTable("estimated");
    private final IntegerSubscriber estimateIDSub = estimateTable.getIntegerTopic("id").subscribe(-1);
    private final DoubleSubscriber estimateXSub = estimateTable.getDoubleTopic("x").subscribe(-1);
    private final DoubleSubscriber estimateYSub = estimateTable.getDoubleTopic("y").subscribe(-1);
    private final DoubleSubscriber estimateWSub = estimateTable.getDoubleTopic("w").subscribe(-1);

    private int resetFlag = 0;
    private final IntegerPublisher resetFlagPub = odomTable.getIntegerTopic("flag").publish();

    int estimateListenerHandle;

    public FishServerNT(
        Consumer<FilterEstimate> estimateListener
    ) {
        estimateListenerHandle = inst.addListener(
            estimateIDSub, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                estimateListener.accept(new FilterEstimate(
                    (int) estimateIDSub.get(),
                    new Pose2d(
                        new Translation2d(estimateXSub.get(), estimateYSub.get()),
                        new Rotation2d(estimateWSub.get())
                    )
                ));
            }
        );
    }

    public void publish(SendableOdomMeasurement odometry) {
        publish(odometry, new SendableVisionMeasurement(odometry.getId()));
        // FIXME somehow tell the PF to only update odom data (possible fix above, needs to be tested)
    }

    public void publish(SendableOdomMeasurement odometry, SendableVisionMeasurement vision) {
        resetFlagPub.set(resetFlag);
        // "I don't trust like that!"
        if (odometry.getId() == vision.getMeasID()) {
            odomIDPub.set(odometry.getId());
            visionIDPub.set(vision.getMeasID());
        } else {
            odomIDPub.set(odometry.getId());
            visionIDPub.set(odometry.getId());
        }
        odomXPub.set(odometry.getPose().getX());
        odomYPub.set(odometry.getPose().getY());
        odomWPub.set(odometry.getPose().getRotation().getRadians());
        hasTargetsPub.set(vision.hasTargets());
        tagIDPub.set(vision.getTagID());
        distancePub.set(vision.getDistance());
        distanceXPub.set(vision.getDistanceX());
        distanceYPub.set(vision.getDistanceY());
        camposePub.set(vision.getCamPose());
    }

    public void addOne() {
        ++resetFlag;
    }
}
