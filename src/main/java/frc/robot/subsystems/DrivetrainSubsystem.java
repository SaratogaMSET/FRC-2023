package frc.robot.subsystems;

import frc.lib.swerve.BetterSwerveModuleState;
import frc.lib.swerve.SwerveDriveKinematics2;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainUtil.SwerveModuleIOInputsAutoLogged;

public class DrivetrainSubsystem extends SubsystemBase {
    private BetterSwerveModuleState[] currentDesiredState = new BetterSwerveModuleState[4];
    private BetterSwerveModuleState[] previousDesiredState = new BetterSwerveModuleState[4];

    public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    public double offset = 0;
    private Rotation2d lastRotation = new Rotation2d();
    private final PIDController driftCorrectionPID = new PIDController(0.225, 0.00, 0.005);
    public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);    
    private SwerveDriveOdometry swerveOdometry;
    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[] {new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()};
    double pXY = 0;
    double desiredHeading;

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
        SmartDashboard.putNumber("navX Offset", offset);
        return Math.toRadians(angle);
      }

      public void drive(ChassisSpeeds chassisSpeeds) {
        double IpX = Units.inchesToMeters(2);
        double IpY = Units.inchesToMeters(-5.4);
        double IpD = Math.hypot(IpY, IpX);
        double theta = Math.atan2(IpY, IpX);
        double omega = chassisSpeeds.omegaRadiansPerSecond;
        double deltaTheta = 0.02 * omega;
        chassisSpeeds.vxMetersPerSecond += IpD * omega * (Math.cos(theta)*Math.cos(deltaTheta) - Math.sin(theta)*Math.sin(deltaTheta));
        chassisSpeeds.vyMetersPerSecond += IpD * omega * (Math.sin(theta)*Math.cos(deltaTheta) + Math.cos(theta)*Math.sin(deltaTheta));
        chassisSpeeds = driftCorrection(chassisSpeeds);
        m_chassisSpeeds = chassisSpeeds;
      }    

      public ChassisSpeeds driftCorrection(ChassisSpeeds speeds){
        double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
        
        if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0) desiredHeading = getPose().getRotation().getDegrees();
        
        else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(getPose().getRotation().getDegrees(), desiredHeading);
        
        pXY = xy;
        return speeds;
        }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        BetterSwerveModuleState[] desiredStates = Constants.Drivetrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);
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
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
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

        Logger.getInstance().recordOutput("CurrentSwerveModuleStates", getModuleStates());
        for(int i =0; i < 4; i++){
            mSwerveMods[i].updateInputs(moduleInputs[i]);
        }
        Logger.getInstance().recordOutput("Before Correction", Constants.Drivetrain.m_kinematics.toSwerveModuleStates(m_chassisSpeeds));
        ChassisSpeeds speeds = driftCorrection(m_chassisSpeeds);
        Logger.getInstance().recordOutput("After Correction", Drivetrain.m_kinematics.toSwerveModuleStates(speeds));

        previousDesiredState = currentDesiredState;
        currentDesiredState = Constants.Drivetrain.m_kinematics.toSwerveModuleStates(speeds);

        // ChassisSpeeds currentSpeeds = Constants.Drivetrain.m_kinematics.toChassisSpeeds(previousDesiredState);
        SwerveDriveKinematics2.desaturateWheelSpeeds(currentDesiredState, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

        if(previousDesiredState[0] == null || previousDesiredState[1] == null || previousDesiredState[2] == null || previousDesiredState[3] == null){
            previousDesiredState = currentDesiredState;
        }

        double joystickCenterState = 0;
        boolean joystickCentered = true;
        for(int i = 0; i < currentDesiredState.length && joystickCentered; i++){
            if(Math.abs(currentDesiredState[i].angle.getRadians() - joystickCenterState) > 0.001) joystickCentered = false;
        }
        
        if(joystickCentered){
            currentDesiredState[0].angle = previousDesiredState[0].angle;
            currentDesiredState[1].angle = previousDesiredState[1].angle;
            currentDesiredState[2].angle = previousDesiredState[2].angle;
            currentDesiredState[3].angle = previousDesiredState[3].angle;
        }
        for(int i = 0; i < 4; i++){
            SmartDashboard.putNumber("Mod " + i + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Integrated", mSwerveMods[i].getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Desired Module "+ i + " Angle", currentDesiredState[i].angle.getDegrees());   
            SmartDashboard.putNumber("Desired Module "+ i + " Velocity", currentDesiredState[i].speedMetersPerSecond);     
        }
        
        setModuleStates(currentDesiredState);
        Logger.getInstance().recordOutput("Gyro Rotation2d", new Pose2d(new Translation2d(0,0),m_navx.getRotation2d()));
        Logger.getInstance().recordOutput("Gyro Yaw", m_navx.getYaw());
        swerveOdometry.update(getRotation2d(), getModulePositions());  
        Logger.getInstance().recordOutput("Odometry", swerveOdometry.getPoseMeters());
        // lastRotation = new Rotation2d(-gyroInputs.yawPositionRad);
    }
}
