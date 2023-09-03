package frc.robot.subsystems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeomUtil;
import frc.lib.swerve.BetterSwerveModuleState;
import frc.lib.swerve.SwerveDriveKinematics2;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private BetterSwerveModuleState[] currentDesiredState = new BetterSwerveModuleState[4];
    private BetterSwerveModuleState[] previousDesiredState = new BetterSwerveModuleState[4];

    private SwerveModuleState[] currentDesiredStateTele = new SwerveModuleState[4];
    private SwerveModuleState[] previousDesiredStateTele = new SwerveModuleState[4];
    
    public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); //this is our gyro that we use
    public double offset = 0; //offset that we add to every time we rezero the gyro
    private final PIDController driftCorrectionPID = new PIDController(0.1, 0.00, 0.000); //a pid controller to fight drift

    //the speeds that we use to convert to individual swerve module states
    public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);    
    public ChassisSpeeds previouChassisSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
    ChassisSpeeds speeds = new ChassisSpeeds(0.0,0.0,0.0);

    private NetworkTable visionData;

    /*How much do we "trust" our drivetrain measurements and how much do we "trust" our vision estimates. 
     * A higher value means we trust it more.
     * */
    private final Matrix<N3, N1> stateSTD = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N1> visDataSTD = new Matrix<>(Nat.N3(), Nat.N1());

    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[] {new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()};
    double pXY = 0;
    double desiredHeading;
    Pose2d lastPose;
    //Normal motor encoder data for odometry combined with vision estimates
    SwerveDrivePoseEstimator odomFiltered = new SwerveDrivePoseEstimator(Constants.Drivetrain.m_kinematics2, getRotation2d(), getModulePositions(), lastPose);
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

    
    /**simple conversion from what the navx angle is to a rotation 2d
     * Note that navx considers clockwise to be positive, while our coordinate system is clockwise is negative
     */
    public Rotation2d getRotation2d() {
        // if(gyroInputs.connected){
            return Rotation2d.fromDegrees(-Math.toDegrees(getNavHeading()));
        // }
        // else {
        //     return Rotation2d.fromRadians(m_chassisSpeeds.omegaRadiansPerSecond * 0.02 + lastRotation.getRadians());
        // }
    }
    //Get the reading of the navx in radians
    public double getNavHeading(){
        double angle = m_navx.getYaw() + 180 - offset;
        angle = angle + 90 + 360;
        angle = angle % 360;
        return Math.toRadians(angle);
      }


      /**Use this to command chassis speeds, which will be converted
       * Note: In teleop we use a Pose2d.log(), which will allow us to generate a curve that the robot will follow
       * instead of the direct chassis speeds. We do this to counteract the robot skewing in the direction of rotation
       * while moving and rotating at the same time.
       * 
       * 2nd order kinematics also does this, but this is only viable for auton as it makes assumptions (wont get into the math here)
       * that make it unviable for teleop
       * @param chassisSpeeds the desired ChassisSpeeds
       */
      public void drive(ChassisSpeeds chassisSpeeds) {
        // double IpX = Units.inchesToMeters(-2); //TODO: Replace with constant
        // double IpY = Units.inchesToMeters(-5.4); //TODO: Replace with constant
        // double IpD = Math.hypot(IpY, IpX);
        // double theta = Math.atan2(IpY, IpX);
        // double omega = chassisSpeeds.omegaRadiansPerSecond;
        // double deltaTheta = 0.02 * omega;
        // chassisSpeeds.vxMetersPerSecond += IpD * omega * (Math.cos(theta)*Math.cos(deltaTheta) - Math.sin(theta)*Math.sin(deltaTheta));
        // chassisSpeeds.vyMetersPerSecond += IpD * omega * (Math.sin(theta)*Math.cos(deltaTheta) + Math.cos(theta)*Math.sin(deltaTheta));
        
        // chassisSpeeds = driftCorrection(chassisSpeeds);

        if(DriverStation.isTeleop()){
            Pose2d velocity = new Pose2d(chassisSpeeds.vxMetersPerSecond * Constants.loopPeriodSecs,
                chassisSpeeds.vyMetersPerSecond * Constants.loopPeriodSecs,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * Constants.loopPeriodSecs));
            Twist2d twist_vel = GeomUtil.log(velocity);
            m_chassisSpeeds = new ChassisSpeeds(twist_vel.dx / Constants.loopPeriodSecs, 
            twist_vel.dy / Constants.loopPeriodSecs, 
            twist_vel.dtheta / Constants.loopPeriodSecs);
        }
        else{
            m_chassisSpeeds = chassisSpeeds;
        }
      }    

      public ChassisSpeeds driftCorrection(ChassisSpeeds speeds){
        double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
        
        if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0) desiredHeading = lastPose.getRotation().getDegrees();
        
        else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(lastPose.getRotation().getDegrees(), desiredHeading);
        
        pXY = xy;
        return speeds;
        }

    /**We use 2nd order kinematics for auton, and this is converting the desired speed of the robot to individual states 
     * each swerve module will go to.
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {

        BetterSwerveModuleState[] desiredStates = Constants.Drivetrain.m_kinematics2.toSwerveModuleStates(chassisSpeeds);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /**To directly command the states to the modules
     * @param desiredStates 2nd order kinematics states
     */
    public void setModuleStates(BetterSwerveModuleState[] desiredStates) {
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**To directly command the states to the modules
     * @param desiredStates 1st order kinematics states
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    /**Get the odom position from the field */
    public Pose2d getPose() {
        return odomFiltered.getEstimatedPosition();
    }

    /** Reset the odom to a given pose, useful for when starting auton period */
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

            case SIM:
                for(int i = 0; i < 4; i++){
                    states[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return states;

            case REPLAY:
                for(int i = 0; i < 4; i++){
                    states[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
            return states;

            default:
                for(int i = 0; i < 4; i++){
                    states[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
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
        
            case SIM:
                for(int i = 0; i < mSwerveMods.length; i ++){

                    positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return positions;
            case REPLAY:
                for(int i = 0; i < mSwerveMods.length; i ++){
                
                    positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return positions;
            default:
                for(int i = 0; i < mSwerveMods.length; i ++){
                
                    positions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
                }
                return positions;
        }
    }

    public void zeroGyroscope(){
        //Pose2d pose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
        m_navx.zeroYaw();
        resetOdometry(getPose());
      
    }

    public Rotation2d getYaw() {
        return (Constants.Drivetrain.invertGyro) ? Rotation2d.fromDegrees(360 - m_navx.getYaw()) : Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /** Instead of getting a robot relative velocity, we want to get a field velocity: useful for auto scoring in the future */
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

        //We only want to use 2nd order kinematics during auton, otherwise we trace the arc that the robot wants to go
        //based on the desired chassis speeds and 1st order kinematics and command that instead to the modules
        //See drive() method in this class 

        if(DriverStation.isAutonomous()){
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

            //useful debugging

            // for(int i = 0; i < 4; i++){
            //     SmartDashboard.putNumber("Mod " + i + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
            // //     SmartDashboard.putNumber("Mod " + i + " Integrated", mSwerveMods[i].getPosition().angle.getDegrees());
            // //     SmartDashboard.putNumber("Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
            // //     SmartDashboard.putNumber("Desired Module "+ i + " Angle", currentDesiredState[i].angle.getDegrees());   
            // //     SmartDashboard.putNumber("Desired Module "+ i + " Velocity", currentDesiredState[i].speedMetersPerSecond);     
            // }
            
            Logger.getInstance().recordOutput("DesiredSwerveModuleStatesAutonomous", currentDesiredState);
            setModuleStates(currentDesiredState);
        }
        else{
            previousDesiredStateTele = currentDesiredStateTele;
            currentDesiredStateTele = Constants.Drivetrain.m_kinematics1.toSwerveModuleStates(m_chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(currentDesiredStateTele, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
            
            if(previousDesiredStateTele[0] == null || previousDesiredStateTele[1] == null || previousDesiredStateTele[2] == null || previousDesiredStateTele[3] == null){
                previousDesiredStateTele = currentDesiredStateTele;
            }

            double joystickCenterState = 0;
            boolean joystickCentered = true;
            for(int i = 0; i < currentDesiredStateTele.length && joystickCentered; i++){
                if(Math.abs(currentDesiredStateTele[i].angle.getRadians() - joystickCenterState) > 0.004) joystickCentered = false;
            }
            
            if(joystickCentered){
                currentDesiredStateTele[0].angle = previousDesiredStateTele[0].angle;
                currentDesiredStateTele[1].angle = previousDesiredStateTele[1].angle;
                currentDesiredStateTele[2].angle = previousDesiredStateTele[2].angle;
                currentDesiredStateTele[3].angle = previousDesiredStateTele[3].angle;
            }

            // for(int i = 0; i < 4; i++){
            //     SmartDashboard.putNumber("Mod " + i + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
            // //     SmartDashboard.putNumber("Mod " + i + " Integrated", mSwerveMods[i].getPosition().angle.getDegrees());
            // //     SmartDashboard.putNumber("Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
            // //     SmartDashboard.putNumber("Desired Module "+ i + " Angle", currentDesiredState[i].angle.getDegrees());   
            // //     SmartDashboard.putNumber("Desired Module "+ i + " Velocity", currentDesiredState[i].speedMetersPerSecond);     
            // }
            
            Logger.getInstance().recordOutput("DesiredSwerveModuleStatesTele", currentDesiredStateTele);
            setModuleStates(currentDesiredStateTele);
        }
        
        // odomFiltered.update(getRotation2d(), new Rotation2d(m_navx.getPitch()), new Rotation2d(m_navx.getRoll()), getModulePositions());
        odomFiltered.update(getRotation2d(), getModulePositions());

        // double[] pos = new double[]{odomFiltered.getEstimatedPosition().getX(), odomFiltered.getEstimatedPosition().getY(), odomFiltered.getEstimatedPosition().getRotation().getDegrees()};
        // SmartDashboard.putNumberArray("Odom", pos);

        //we dont want to ruin our estimator by feeding in a null value
        lastPose = odomFiltered.getEstimatedPosition();
        Pose2d pose = getVisionPose2d();
        if (pose != null && DriverStation.isTeleop()){
            double timestamp = Timer.getFPGATimestamp() - (visionData.getEntry("tl").getDouble(0) + 11) / 1000;
            odomFiltered.addVisionMeasurement(pose, timestamp);
        }
        //swerveOdometry.update(getRotation2d(), getModulePositions());  
        Logger.getInstance().recordOutput("UKF Odometry", odomFiltered.getEstimatedPosition());
        
    }
}