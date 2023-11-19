package frc.robot.subsystems.Vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseSmoothingFilter extends SubsystemBase {

    private Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private Supplier<Rotation2d> rotationSupplier;
    private Supplier<Double> timestampSupplier;

    private Supplier<Optional<Pose2d>> visionPoseData;    
    private final SwerveDrivePoseEstimator odomFiltered;
    private final SwerveDriveOdometry odometry;

    private final Field2d m_field = new Field2d();

    private final double THRESHOLD = 0.9;
    private final double FREQUENCY = 50.0; // how often the thing's updated I think??
    private boolean ready = false;
    private double timerInterval = 1000.0 / FREQUENCY;
    private double latency = 25.0;
    private double strength = 5.0;
    private double multiplier = 1.0;
    private double smoothingOffsetX = 0.0;
    private double smoothingOffsetY = 1.0;
    private double weight;
    private Pose2d position;
    private boolean seeded;

    public PoseSmoothingFilter(Supplier<SwerveModulePosition[]> modulePositionSupplier, Supplier<Rotation2d> rotationSupplier, Supplier<Optional<Pose2d>> visionPoseData, Supplier<Double> timestampSupplier) {

        this.modulePositionSupplier = modulePositionSupplier;
        this.rotationSupplier = rotationSupplier;
        this.seeded = false;
        this.timestampSupplier = timestampSupplier;

        this.odomFiltered = new SwerveDrivePoseEstimator(Constants.Drivetrain.m_kinematics2,
            rotationSupplier.get(), 
            modulePositionSupplier.get(),
            new Pose2d(),
            Constants.Vision.stateSTD, 
            Constants.Vision.visDataSTD); 

        this.odometry = new SwerveDriveOdometry(Constants.Drivetrain.m_kinematics2, rotationSupplier.get(), modulePositionSupplier.get());

        this.visionPoseData = visionPoseData;
    }

    public Pose2d getPose() {
        return odomFiltered.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        ready = false;

        odomFiltered.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), pose);
        odometry.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), pose);
    }

    private Pose2d filter(Pose2d calcTarget) {
        if (!ready) {
            position = calcTarget;
            setWeight(latency);
            ready = true;
            return calcTarget;
        }

        var delta = new Pose2d(calcTarget.getX() - position.getX(), calcTarget.getY() - position.getY(),
                new Rotation2d(calcTarget.getRotation().getRadians() - position.getRotation().getRadians()));
        var distance = Math.hypot(calcTarget.getX() - position.getX(), calcTarget.getY() - position.getY());

        // Someone tell me how and why this works.
        var weightModifier = Math.pow(distance + smoothingOffsetX, strength * -1) * multiplier;

        // Limit minimum
        if (weightModifier + smoothingOffsetY < 0) {
            weightModifier = 0;
        } else {
            weightModifier += smoothingOffsetY;
        }

        weightModifier = weight / weightModifier;
        weightModifier = MathUtil.clamp(weightModifier, 0, 1);
        position = new Pose2d(position.getX() + delta.getX() * weightModifier,
            position.getY() + delta.getY() * weightModifier, new Rotation2d(
                position.getRotation().getRadians() + delta.getRotation().getRadians() * weightModifier));
        
        return position;
    }

    private void setWeight(double latency) {
        double stepCount = latency / timerInterval;
        double target = 1 - THRESHOLD;
        weight = 1.0 - (1.0 / Math.pow(1.0 / target, 1.0 / stepCount));
    }

    
    @Override
    public void periodic() {

        var pose = visionPoseData.get();
        
        if (seeded == false && pose.isPresent()){
            seeded = true;
            odomFiltered.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), pose.get());
            odometry.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), pose.get());
            SmartDashboard.putNumberArray("Seed Pose", new double[]{pose.get().getX(), pose.get().getY()});
        } else {

            if (pose.isPresent() && DriverStation.isTeleop() && 
                getPose().getTranslation().getDistance(pose.get().getTranslation()) < 0.5){
                    double time = this.timestampSupplier.get();
                    var data = pose.get();
                    odomFiltered.addVisionMeasurement(data, time);
                    SmartDashboard.putNumberArray("Vision Poses", new double[]{data.getX(), data.getY()});
            } 
            
        }

        odomFiltered.update(rotationSupplier.get(), modulePositionSupplier.get());
        odometry.update(rotationSupplier.get(), modulePositionSupplier.get());

        var dummyVar = getPose();   
        
        SmartDashboard.putNumberArray("Raw Localizer pose", new double[]{
            dummyVar.getX(), dummyVar.getY(), dummyVar.getRotation().getRadians()
        });

        //m_field.setRobotPose(position.getX(), position.getY(), position.getRotation());

        m_field.setRobotPose(odomFiltered.getEstimatedPosition());

        var delta = odomFiltered.getEstimatedPosition().getTranslation().minus(Constants.Vision.apriltag7.getTranslation().toTranslation2d());
        var delta2 = odometry.getPoseMeters().getTranslation().minus(Constants.Vision.apriltag7.getTranslation().toTranslation2d());

        double[] test = new double[]{Units.metersToInches(delta.getX()), Units.metersToInches(delta.getY())};
        double[] test1 = new double[]{delta.getX(), delta.getY()};

        double[] test3 = new double[]{Units.metersToInches(delta2.getX()), Units.metersToInches(delta2.getY())};
        double[] test4 = new double[]{delta2.getX(), delta2.getY()};
        

        SmartDashboard.putNumberArray("Delta to Pose", test);
        SmartDashboard.putNumberArray("Delta to Pose(meters)", test1);

        SmartDashboard.putNumberArray("Delta to VisionLess Odom", test3);
        SmartDashboard.putNumberArray("Delta to  VisionLess Odom", test4);

        SmartDashboard.putData(m_field);
    }
}

