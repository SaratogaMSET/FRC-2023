package frc.robot.subsystems.Drivetrain;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.lib.swerve.SwerveDriveKinematics2;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private SwerveModuleState[] currentDesiredState = new SwerveModuleState[4];
    private SwerveModuleState[] previousDesiredState = new SwerveModuleState[4];

    public double offset = 0;
    private Rotation2d lastRotation = new Rotation2d();
    private final PIDController driftCorrectionPID = new PIDController(0.1, 0.00, 0.000);
    public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);    
    //private SwerveDriveOdometry swerveOdometry;
    public ChassisSpeeds previouChassisSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
    ChassisSpeeds speeds = new ChassisSpeeds(0.0,0.0,0.0);
    private Field2d m_field = new Field2d();

    private Supplier<Optional<EstimatedRobotPose>> visionPoseData;   
    private Supplier<Double> timestampSupplier;
    private Supplier<Vector<N3>> stdDevsSupplier;

    private final SwerveDrivePoseEstimator odomFiltered;
    private final SwerveDriveOdometry odometry;
    private boolean seeded;
    

    public GyroIO gyroIO;
    public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    public double simHeading = 0.0;

    public SwerveModuleIOInputsAutoLogged[] inputs;

    double pXY = 0;
    double desiredHeading;
    Pose2d lastPose;

    public SwerveModuleIO[] mSwerveMods; 
    
    public DrivetrainSubsystem(SwerveModuleIO[] swerveIO, GyroIO gyroIO, Supplier<Optional<EstimatedRobotPose>> visionPoseData, Supplier<Double> timestampSupplier, Supplier<Vector<N3>> stddevs) {  
        this.gyroIO = gyroIO;
        // m_navx.zeroYaw();
        //zeroGyroscope();  
        mSwerveMods = swerveIO;
        inputs = new SwerveModuleIOInputsAutoLogged[] {
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged()
          };
        resetModulesToAbsolute();    


        // Vision Section
        this.visionPoseData = visionPoseData;
        this.odomFiltered = new SwerveDrivePoseEstimator(Constants.Drivetrain.m_kinematics1,
            getRotation2d(), 
            getModulePositions(),
            new Pose2d(),
            Constants.Vision.stateSTD, 
            Constants.Vision.visDataSTD);

        this.odometry = new SwerveDriveOdometry(Constants.Drivetrain.m_kinematics1, getRotation2d(), getModulePositions());
        this.seeded = false;
        this.timestampSupplier = timestampSupplier;
        this.stdDevsSupplier = stddevs;

        //swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.m_kinematics, getRotation2d(), getModulePositions());   
    }
    
    public Rotation2d getRotation2d() {
        // if(gyroInputs.connected){
            if (Robot.isReal()) {
                return (Constants.Drivetrain.invertGyro)
                    ? Rotation2d.fromDegrees(360 - gyroIO.getHeading().getDegrees())
                    : Rotation2d.fromDegrees(gyroIO.getHeading().getDegrees());
              }
              return Rotation2d.fromDegrees(simHeading);
        // }
        // else {
        //     return Rotation2d.fromRadians(m_chassisSpeeds.omegaRadiansPerSecond * 0.02 + lastRotation.getRadians());
        // }
    }
    public double getNavHeading(){
        double angle = (gyroIO.getHeading()).getDegrees() + 180 - offset;
        angle = angle + 90 + 360;
        angle = angle % 360;
        return Math.toRadians(angle);
      }

      public void drive(ChassisSpeeds chassisSpeeds) {
        // double IpX = Units.inchesToMeters(2);
        // double IpY = Units.inchesToMeters(-5.4);
        // double IpD = Math.hypot(IpY, IpX);
        // double theta = Math.atan2(IpY, IpX);
        // double omega = chassisSpeeds.omegaRadiansPerSecond;
        // double deltaTheta = 0.02 * omega;
        // chassisSpeeds.vxMetersPerSecond += IpD * omega * (Math.cos(theta)*Math.cos(deltaTheta) - Math.sin(theta)*Math.sin(deltaTheta));
        // chassisSpeeds.vyMetersPerSecond += IpD * omega * (Math.sin(theta)*Math.cos(deltaTheta) + Math.cos(theta)*Math.sin(deltaTheta));
        
        // chassisSpeeds = driftCorrection(chassisSpeeds);
        Pose2d velPose = new Pose2d(new Translation2d(chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond*0.02), new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
        Twist2d velTwist = new Pose2d().log(velPose);
        ChassisSpeeds arcVelocity = new ChassisSpeeds( velTwist.dx / 0.02,
        velTwist.dy / 0.02,
        velTwist.dtheta / 0.02);
        m_chassisSpeeds = arcVelocity;
      }    
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {

        BetterSwerveModuleState[] desiredStates = Constants.Drivetrain.m_kinematics2.toSwerveModuleStates(chassisSpeeds);
        for(SwerveModuleIO mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for(SwerveModuleIO mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public Pose2d getPose() {
        return odomFiltered.getEstimatedPosition();
        //return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odomFiltered.resetPosition(getRotation2d(), getModulePositions(), pose);
        //swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setX(){
        setModuleStates(new BetterSwerveModuleState[]{
            new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(45), 0),
            new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(-45), 0),
            new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(-45), 0),
            new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(45), 0),
        });
    }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i =0; i < 4; i++){
                    states[i] = mSwerveMods[i].getState();
                }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
                for(int i = 0; i < mSwerveMods.length; i ++){
                    positions[i] = mSwerveMods[i].getPosition();
                        // positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return positions;
        }

    public void zeroGyroscope(){
        //Pose2d pose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
        gyroIO.resetHeading();
        simHeading = 0.0;
        resetOdometry(getPose());
      
    }

    public static double apply(double xCoordinate) {
        if (DriverStation.getAlliance() == Alliance.Red) {
          return Units.inchesToMeters(651.25) - xCoordinate;
        } else {
          return xCoordinate;
        }
      }

    public Rotation2d getYaw() {
        return (Constants.Drivetrain.invertGyro) ? Rotation2d.fromDegrees(360 - gyroIO.getHeading().getDegrees()) : (gyroIO.getHeading());
    }

    public Rotation2d getRoll(){
        return gyroIO.getRoll();
    }

    public Rotation2d getPitch(){
        return gyroIO.getPitch();
    }

    public double getWorldLinearAccelX(){
        return gyroIO.getWorldLinearAccelX();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleIO mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Twist2d getFieldVelocity(){
        ChassisSpeeds chassisSpeeds = Drivetrain.m_kinematics2.toChassisSpeeds(getModuleStates());
        Translation2d linearFieldVelocity =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getRotation2d());
        
        return new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                chassisSpeeds.omegaRadiansPerSecond);
    }
    

    public ChassisSpeeds limitVelocity(ChassisSpeeds chassisSpeeds){
        speeds.vxMetersPerSecond = Math.max(-Drivetrain.allowedMaxAcceleration, Math.min(Drivetrain.allowedMaxAcceleration, chassisSpeeds.vxMetersPerSecond));
        speeds.vyMetersPerSecond = Math.max(-Drivetrain.allowedMaxAcceleration, Math.min(Drivetrain.allowedMaxAcceleration, chassisSpeeds.vyMetersPerSecond));
        speeds.omegaRadiansPerSecond = Math.max(-Drivetrain.allowedMaxAcceleration, Math.min(Drivetrain.allowedMaxAcceleration, chassisSpeeds.omegaRadiansPerSecond));
        return speeds;
    }

    @Override
    public void periodic(){
        for (int i = 0; i < mSwerveMods.length; i++) {
            inputs[i] = mSwerveMods[i].updateInputs();
            Logger.getInstance().processInputs("Swerve Module " + i, inputs[i]);
          }
      
          gyroInputs = gyroIO.updateInputs();
          Logger.getInstance().processInputs("Gyro", gyroInputs);
          simHeading += Units.radiansToDegrees(m_chassisSpeeds.omegaRadiansPerSecond);

        Logger.getInstance().recordOutput("CurrentSwerveModuleStates", getModuleStates());

        // for(int i = 0; i < 4; i++){
        //     // mSwerveMods[i].updateInputs(moduleInputs[i]);
        //     // SmartDashboard.putNumber(i + "cancoder", mSwerveMods[i].getCanCoder().getDegrees());
        //     // SmartDashboard.putNumber(i + "Motor Output Percent", mSwerveMods[i].getOutputPercent());
        //     // SmartDashboard.putNumber(i + "Motor Output Voltage", mSwerveMods[i].getOutputVoltage());
        //     // SmartDashboard.putNumber(i + "Velocity", mSwerveMods[i].getVelocity());
        //     // SmartDashboard.putNumber(i + "RPM" , mSwerveMods[i].getRPM());
        // //     SmartDashboard.putNumber("Mod " + i + " Integrated", mSwerveMods[i].getPosition().angle.getDegrees());
        // //     SmartDashboard.putNumber("Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
        // //     SmartDashboard.putNumber("Desired Module "+ i + " Angle", currentDesiredState[i].angle.getDegrees());   
        // //     SmartDashboard.putNumber("Desired Module "+ i + " Velocity", currentDesiredState[i].speedMetersPerSecond); 
        //     // Logger.getInstance().recordOutput(i + "Motor Output Percent", mSwerveMods[i].getOutputPercent());
        //     // Logger.getInstance().recordOutput(i + "Motor Output Voltage", mSwerveMods[i].getOutputVoltage());

        // }

        previousDesiredState = currentDesiredState;
        currentDesiredState = Constants.Drivetrain.m_kinematics1.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(currentDesiredState, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

        if(previousDesiredState[0] == null || previousDesiredState[1] == null || previousDesiredState[2] == null || previousDesiredState[3] == null){
            previousDesiredState = currentDesiredState;
        }

        double joystickCenterState = 0;
        boolean joystickCentered = true;
        for(int i = 0; i < currentDesiredState.length && joystickCentered; i++){
            if(Math.abs(currentDesiredState[i].angle.getRadians() - joystickCenterState) > 0.004) joystickCentered = false;
        }
        
        if(joystickCentered){
            currentDesiredState[0].angle = previousDesiredState[0].angle;
            currentDesiredState[1].angle = previousDesiredState[1].angle;
            currentDesiredState[2].angle = previousDesiredState[2].angle;
            currentDesiredState[3].angle = previousDesiredState[3].angle;
        }

        setModuleStates(currentDesiredState);
        //Logger.getInstance().recordOutput("Gyro Rotation2d", new Pose2d(new Translation2d(0,0),m_navx.getRotation2d()));
        //Logger.getInstance().recordOutput("Gyro Yaw", m_navx.getYaw());



        var pose = visionPoseData.get();
        
        if (seeded == false && pose.isPresent()){
            seeded = true;
            odomFiltered.resetPosition(getRotation2d(), getModulePositions(), pose.get().estimatedPose.toPose2d());
            odometry.resetPosition(getRotation2d(), getModulePositions(), pose.get().estimatedPose.toPose2d());
            SmartDashboard.putNumberArray("Seed Pose", new double[]{pose.get().estimatedPose.toPose2d().getTranslation().getX(),pose.get().estimatedPose.toPose2d().getTranslation().getY()});
        } else {

            if (pose.isPresent() && DriverStation.isTeleop() && 
                getPose().getTranslation().getDistance(pose.get().estimatedPose.toPose2d().getTranslation()) < 0.5){
                    double time = this.timestampSupplier.get();
                    var data = pose.get();
                    odomFiltered.addVisionMeasurement(data.estimatedPose.toPose2d(), time, stdDevsSupplier.get());
                    SmartDashboard.putNumberArray("Vision Poses", new double[]{data.estimatedPose.toPose2d().getTranslation().getX(), data.estimatedPose.toPose2d().getTranslation().getY()});
            } 
            
        }

        if (Robot.isReal()) {
            odomFiltered.update(getRotation2d(), getModulePositions());
            odometry.update(getRotation2d(), getModulePositions());
           } else {
             odomFiltered.update(Rotation2d.fromDegrees(simHeading), getModulePositions());
             odometry.update(getRotation2d(), getModulePositions());
           }
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

        Logger.getInstance().recordOutput("UKF Odometry", odomFiltered.getEstimatedPosition());
        Logger.getInstance().recordOutput("Visionless Odometry", odometry.getPoseMeters());
 
 

        SmartDashboard.putData(m_field);
    }
}