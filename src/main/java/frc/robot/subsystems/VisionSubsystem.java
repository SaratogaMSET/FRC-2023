package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final Field2d rawField;
    private final Field2d filterField;
    // private final ParticleFilter filter;
    // private final Particle robotParticle;
    // private final Runnable resampler;
    // private final ScheduledExecutorService executor;

    private Pose2d rawPose;

    public VisionSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        rawField = new Field2d();
        filterField = new Field2d();
        // filter = new ParticleFilter(
        //     Constants.FilterConstants.NUM_PARTICLES, 
        //     Constants.VisionConstants.Field.TAGS, 
        //     Constants.VisionConstants.Field.FIELD_WIDTH, 
        //     Constants.VisionConstants.Field.FIELD_HEIGHT
        // );
        // robotParticle = new Particle(
        //     Constants.VisionConstants.Field.TAGS,
        //     Constants.VisionConstants.Field.FIELD_WIDTH, 
        //     Constants.VisionConstants.Field.FIELD_HEIGHT
        // );

        // initFilter();

        SmartDashboard.putData("Raw Robot Position", rawField);
        SmartDashboard.putData("Filtered Robot Position", filterField);

        // resampler = new Runnable() {
        //     public void run() {
        //         double[] tempPose = getBotPose();
        //         if (tempPose.length > 1) {
        //             float x = (float) tempPose[0];
        //             float y = (float) tempPose[1];
        
        //             robotParticle.set(x, y, 0, 0); // TODO convert Euler angles to radians --> orientation
        //             filter.resample(robotParticle.sense());
        //         }
        //     }
        // };
        // executor = Executors.newScheduledThreadPool(1);
        // executor.scheduleAtFixedRate(resampler, 0, 2, TimeUnit.SECONDS);
    }

    // private final void initFilter() {
    //     double[] tempPose = getBotPose();
    //     if (tempPose.length > 1) {
    //         float x = (float) tempPose[0];
    //         float y = (float) tempPose[1];

    //         filter.setNoise(0.5f, 0.5f, 5f); // TODO tune
    //         robotParticle.set(x, y, 0, 0); // TODO convert Euler angles to radians --> orientation
    //         filter.resample(robotParticle.sense());
    //     }
    // }

    // private void updateFilter() {
    //     double[] tempPose = getBotPose();
    //     if (tempPose.length > 1) {
    //         float x = (float) tempPose[0];
    //         float y = (float) tempPose[1];

    //         robotParticle.set(x, y, 0, 0);
    //         filter.resample(robotParticle.sense());
    //     }
    // }

    private double[] getCamTran() {
        return table.getEntry("camtran").getDoubleArray(new double[10]);
    }

    private double[] getBotPose() {
        return table.getEntry("botpose").getDoubleArray(new double[10]);
    }

    private int getTV() {
        return (int) table.getEntry("tv").getInteger(0);
    }

    @Override
    public void periodic() {
        double[] camtran = getCamTran();
        SmartDashboard.putNumberArray("camtran", camtran);
        double[] botpose = getBotPose();
        SmartDashboard.putNumberArray("botpose", botpose);
        
        if (botpose.length > 1) {
            rawPose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d());
            rawField.setRobotPose(rawPose);
        }

        // updateFilter();
        
        // if (getTV() == 1) {
        //     filterField.setRobotPose(
        //         new Pose2d(
        //             new Translation2d(filter.getAverageParticle().x, filter.getAverageParticle().y),
        //             new Rotation2d()
        //         )
        //     );
        // }
    }

    @Override
    public void simulationPeriodic() {
        rawField.setRobotPose(new Pose2d());
    }
}
