package frc.robot.subsystems.Drivetrain;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommandA;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private BetterSwerveModuleState[] currentDesiredState = new BetterSwerveModuleState[4];
    private BetterSwerveModuleState[] previousDesiredState = new BetterSwerveModuleState[4];

    public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    public double offset = 0;
    private Rotation2d lastRotation = new Rotation2d();
    private final PIDController driftCorrectionPID = new PIDController(0.1, 0.00, 0.000);
    public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);    
    //private SwerveDriveOdometry swerveOdometry;
    public ChassisSpeeds previouChassisSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
    ChassisSpeeds speeds = new ChassisSpeeds(0.0,0.0,0.0);
    private Field2d m_field = new Field2d();

    private NetworkTable visionData;

    private SwerveDrivePoseEstimator odomFiltered;
    private final Matrix<N3, N1> stateSTD = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N1> visDataSTD = new Matrix<>(Nat.N3(), Nat.N1());


    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[] {new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()};
    double pXY = 0;
    double desiredHeading;
    Pose2d lastPose;

    public SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Drivetrain.Mod0.constants),
        new SwerveModule(1, Constants.Drivetrain.Mod1.constants),
        new SwerveModule(2, Constants.Drivetrain.Mod2.constants),
        new SwerveModule(3, Constants.Drivetrain.Mod3.constants)
    };
    
    public DrivetrainSubsystem() {  
        m_navx.zeroYaw();
        //zeroGyroscope();  
        resetModulesToAbsolute();    
        visionData = NetworkTableInstance.getDefault().getTable("limelight-three");


        stateSTD.set(0, 0, 0.20); stateSTD.set(1, 0, 0.2); stateSTD.set(2, 0, 0.2); //Tune Values
        visDataSTD.set(0, 0, 0.80); visDataSTD.set(1, 0, 0.8); visDataSTD.set(2, 0, 0.8);

        this.odomFiltered = new SwerveDrivePoseEstimator(Constants.Drivetrain.m_kinematics2, getRotation2d(), getModulePositions(), new Pose2d(), stateSTD, visDataSTD);
        //swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.m_kinematics, getRotation2d(), getModulePositions());   
    }

    public Pose2d getVisionPose2d(){
        try{
        visionData = RobotContainer.m_visionSubsystem.getTable();
        }
        catch(Exception e){
            System.out.println(e);
        }
        double[] arr = visionData.getEntry("botpose").getDoubleArray(new double[8]);
        
        if (visionData.getEntry("tv").getDouble(0) == 0) return null;
        else return new Pose2d(new Translation2d(arr[0] + (8.24), arr[1] + 4.065), Rotation2d.fromDegrees(arr[5]));
    }

    
    
    public Rotation2d getRotation2d() {
        // if(gyroInputs.connected){
            return Rotation2d.fromDegrees(-Math.toDegrees(getNavHeading()));
        // }
        // else {
        //     return Rotation2d.fromRadians(m_chassisSpeeds.omegaRadiansPerSecond * 0.02 + lastRotation.getRadians());
        // }
    }
    public double getNavHeading(){
        double angle = m_navx.getYaw() + 180 - offset;
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
        m_chassisSpeeds = chassisSpeeds;
      }    

      public ChassisSpeeds driftCorrection(ChassisSpeeds speeds){
        double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
        
        if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0) desiredHeading = lastPose.getRotation().getDegrees();
        
        else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(lastPose.getRotation().getDegrees(), desiredHeading);
        
        pXY = xy;
        return speeds;
        }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {

        BetterSwerveModuleState[] desiredStates = Constants.Drivetrain.m_kinematics2.toSwerveModuleStates(chassisSpeeds);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleStates(BetterSwerveModuleState[] desiredStates) {
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
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
        switch(Constants.currentMode){

            case REAL:
            for(int i =0; i < 4; i++){
                states[i] = mSwerveMods[i].getState();
            }
            return states;

            // case SIM:
            //     for(int i = 0; i < 4; i++){
            //         states[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            //     }
            //     return states;

            // case REPLAY:
            //     for(int i = 0; i < 4; i++){
            //         states[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            //     }
            // return states;

            default:
            for(int i =0; i < 4; i++){
                states[i] = mSwerveMods[i].getState();
            }
            return states;
        }
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        switch(Constants.currentMode){
            case REAL:
                for(int i = 0; i < mSwerveMods.length; i ++){
                    positions[i] = mSwerveMods[i].getPosition();
                        // positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return positions;
        
            // case SIM:
            //     for(int i = 0; i < mSwerveMods.length; i ++){

            //         positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            //     }
            //     return positions;
            // case REPLAY:
            //     for(int i = 0; i < mSwerveMods.length; i ++){
                
            //         positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            //     }
            //     return positions;
            default:
            for(int i = 0; i < mSwerveMods.length; i ++){
                positions[i] = mSwerveMods[i].getPosition();
                    // positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            }
            return positions;
        }
    }

    public void zeroGyroscope(){
        //Pose2d pose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
        m_navx.zeroYaw();
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
        return (Constants.Drivetrain.invertGyro) ? Rotation2d.fromDegrees(360 - m_navx.getYaw()) : Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
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

        Logger.getInstance().recordOutput("CurrentSwerveModuleStates", getModuleStates());

        // for(int i = 0; i < 4; i++){
        //     // mSwerveMods[i].updateInputs(moduleInputs[i]);
        //     // SmartDashboard.putNumber(i + "cancoder", mSwerveMods[i].getCanCoder().getDegrees());
        //     // SmartDashboard.putNumber(i + "Motor Output Percent", mSwerveMods[i].getOutputPercent());
        //     // SmartDashboard.putNumber(i + "Motor Output Voltage", mSwerveMods[i].getOutputVoltage());
        //     // SmartDashboard.putNumber(i + "Velocity", mSwerveMods[i].getVelocity());
        //     // SmartDashboard.putNumber(i + "RPM" , mSwerveMods[i].getRPM());
        //     // Logger.getInstance().recordOutput(i + "Motor Output Percent", mSwerveMods[i].getOutputPercent());
        //     // Logger.getInstance().recordOutput(i + "Motor Output Voltage", mSwerveMods[i].getOutputVoltage());

        // }
        //Logger.getInstance().recordOutput("Before Correction", Constants.Drivetrain.m_kinematics.toSwerveModuleStates(m_chassisSpeeds));
        //ChassisSpeeds speeds = driftCorrection(m_chassisSpeeds);
        //Logger.getInstance().recordOutput("After Correction", Drivetrain.m_kinematics.toSwerveModuleStates(speeds));

        previousDesiredState = currentDesiredState;
        currentDesiredState = Constants.Drivetrain.m_kinematics2.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics2.desaturateWheelSpeeds(currentDesiredState, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

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

        // for(int i = 0; i < 4; i++){
        //     SmartDashboard.putNumber("Mod " + i + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
        // //     SmartDashboard.putNumber("Mod " + i + " Integrated", mSwerveMods[i].getPosition().angle.getDegrees());
        // //     SmartDashboard.putNumber("Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
        // //     SmartDashboard.putNumber("Desired Module "+ i + " Angle", currentDesiredState[i].angle.getDegrees());   
        // //     SmartDashboard.putNumber("Desired Module "+ i + " Velocity", currentDesiredState[i].speedMetersPerSecond);     
        // }
        

        setModuleStates(currentDesiredState);
        //Logger.getInstance().recordOutput("Gyro Rotation2d", new Pose2d(new Translation2d(0,0),m_navx.getRotation2d()));
        //Logger.getInstance().recordOutput("Gyro Yaw", m_navx.getYaw());

        // odomFiltered.update(getRotation2d(), new Rotation2d(m_navx.getPitch()), new Rotation2d(m_navx.getRoll()), getModulePositions());
        odomFiltered.update(getRotation2d(), getModulePositions());

        // double[] pos = new double[]{odomFiltered.getEstimatedPosition().getX(), odomFiltered.getEstimatedPosition().getY(), odomFiltered.getEstimatedPosition().getRotation().getDegrees()};
        // SmartDashboard.putNumberArray("Odom", pos);
        lastPose = odomFiltered.getEstimatedPosition();
        Pose2d pose = getVisionPose2d();
        if (pose != null && DriverStation.isTeleop()){
            double timestamp = Timer.getFPGATimestamp() - (visionData.getEntry("tl").getDouble(0) + 11) / 1000;
            odomFiltered.addVisionMeasurement(pose, timestamp);
        }

        //swerveOdometry.update(getRotation2d(), getModulePositions());  
        Logger.getInstance().recordOutput("UKF Odometry", odomFiltered.getEstimatedPosition());
        // var asdf = odomFiltered.getEstimatedPosition();
        // SmartDashboard.putNumberArray("UKF Coords", new double[]{asdf.getX(), asdf.getY(), asdf.getRotation().getRadians()});
        //lastRotation = new Rotation2d(-gyroInputs.yawPositionRad);
        m_field.setRobotPose(odomFiltered.getEstimatedPosition());
        //if (pose != null) visionPoseEstimates.setRobotPose(pose);
        // SmartDashboard.putData("Field", m_field);
        // SmartDashboard.putNumber("Nav Heading", getNavHeading());
        //SmartDashboard.putData("visionEstimate", visionPoseEstimates);
        
    }
}