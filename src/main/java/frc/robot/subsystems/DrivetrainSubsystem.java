package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggableChassisSpeeds;
import frc.lib.logging.LoggablePose;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    
    private SwerveModuleState[] currentState = new SwerveModuleState[4];
    private SwerveModuleState[] previousState = new SwerveModuleState[4];
    
    public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    public double offset = 0;
    private LoggableChassisSpeeds SimvSpeeds = new LoggableChassisSpeeds("/SwerveDriveSubsystem/Velocity", new ChassisSpeeds(0.0, 0.0, 0.0));
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveDriveOdometry swerveOdometry;
    private LoggablePose logPose;
    public SwerveModule[] mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Drivetrain.Mod0.constants),
        new SwerveModule(1, Constants.Drivetrain.Mod1.constants),
        new SwerveModule(2, Constants.Drivetrain.Mod2.constants),
        new SwerveModule(3, Constants.Drivetrain.Mod3.constants)
    };

    public DrivetrainSubsystem() {  
        zeroGyroscope();        
        resetModulesToAbsolute();     
        swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.m_kinematics, getRotation2d(), getModulePositions());  
        logPose = new LoggablePose("/SwerveDriveSubsystem/Pose", swerveOdometry.getPoseMeters(),true); 
        
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.toDegrees(getNavHeading()));
    }

    public double getNavHeading(){
        // double angle = m_navx.getFusedHeading() - offset;
        double angle = m_navx.getYaw() + 180 - offset;
        angle = angle + 90 + 360;
        angle = angle % 360;
        // SmartDashboard.putNumber("navX Angle", angle);
        SmartDashboard.putNumber("navX Offset", offset);
        return Math.toRadians(angle);
      }

      public void drive(ChassisSpeeds chassisSpeeds) {

        // double IpX = Units.inchesToMeters(2);
        // double IpY = Units.inchesToMeters(-5.4);
        // double IpD = Math.hypot(IpY, IpX);
        // double theta = Math.atan2(IpY, IpX);
        // double dT = 0.02;
        // double omega = chassisSpeeds.omegaRadiansPerSecond;
        // double deltaTheta = dT * omega;
        // chassisSpeeds.vxMetersPerSecond += IpD * omega * (Math.cos(theta)*Math.cos(deltaTheta) - Math.sin(theta)*Math.sin(deltaTheta));
        // chassisSpeeds.vyMetersPerSecond += IpD * omega * (Math.sin(theta)*Math.cos(deltaTheta) + Math.cos(theta)*Math.sin(deltaTheta));

        SimvSpeeds.set(chassisSpeeds);
        m_chassisSpeeds = chassisSpeeds;
      }    
    
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] desiredStates = Constants.Drivetrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < mSwerveMods.length; i ++){
            positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
    }

    public void zeroGyroscope(){
      m_navx.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Drivetrain.invertGyro) ? Rotation2d.fromDegrees(360 - m_navx.getYaw()) : Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        double[] cSpeeds = new double[]{m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond,m_chassisSpeeds.omegaRadiansPerSecond};
        SmartDashboard.putNumberArray("Desired Chassis Speeds", cSpeeds);

        previousState = currentState;
        currentState = Constants.Drivetrain.m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(currentState, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        
        if(previousState[0] == null || previousState[1] == null || previousState[2] == null || previousState[3] == null){
            previousState = currentState;
        }

        double joystickCenterState = 0;
        boolean joystickCentered = true;
        for(int i = 0; i < currentState.length && joystickCentered; i++){
            if(Math.abs(currentState[i].angle.getRadians() - joystickCenterState) > 0.001) joystickCentered = false;
        }
        
        if(joystickCentered){
            currentState[0].angle = previousState[0].angle;
            currentState[1].angle = previousState[1].angle;
            currentState[2].angle = previousState[2].angle;
            currentState[3].angle = previousState[3].angle;
        }
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Desired Module "+ mod.moduleNumber + " Angle", currentState[mod.moduleNumber].angle.getDegrees());   
            SmartDashboard.putNumber("Desired Module "+ mod.moduleNumber + " Velocity", currentState[mod.moduleNumber].speedMetersPerSecond);     
        }
        setModuleStates(currentState);

        ChassisSpeeds speeds = Constants.Drivetrain.m_kinematics.toChassisSpeeds(currentState);
        double[] realCSpeeds = new double[]{speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond};
        SmartDashboard.putNumberArray("Real Chassis Speeds", realCSpeeds);

        swerveOdometry.update(getRotation2d(), getModulePositions());  
        logPose.set(swerveOdometry.getPoseMeters());
    }
}
