package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmNodeDictionary;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final DoubleSupplier armHeight;
    private final DoubleSupplier actuatorHeight;
    // double minArmValue = 0.3; 
    // double maxArmValue = 1.18; 
    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               DoubleSupplier armHeight,
                               DoubleSupplier actuatorHeight) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.armHeight = armHeight;
        this.actuatorHeight = actuatorHeight;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/6 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/6 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading()+ joyAngle); 

        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;

        double multiplier = Math.pow(Math.abs(m_rotationSupplier.getAsDouble()/9.1), 0.8);

        SmartDashboard.putNumber("m_translationXSupplier", m_translationXTrapezoidal);
        SmartDashboard.putNumber("m_translationYSupplier", m_translationYTrapezoidal);
        SmartDashboard.putNumber("m_rotationSupplier", m_rotationSupplier.getAsDouble());
        SmartDashboard.putNumber("Magnitude", magnitude);
        double armY = armHeight.getAsDouble(); 

        if(armY > 0.3){      
            // if(armHeight.getAsDouble() > minArmValue) {//start slow                 Constants.ArmNodeDictionary.ready_midcube_score_y){
            //     double iLerp = (armY - minArmValue) / (minArmValue - maxArmValue); //inverse linear interpolation to get from 0 to 1 between min and max arm heights
            //     double speedMultiplier = 1 + Math.max(0.0, Math.min(1.0, iLerp)) * (1.5); //clamp it actually between 0 to 1 in case oops! incident
                m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                        resultX/2.5,
                        resultY/2.5,
                        m_rotationSupplier.getAsDouble()/1.5 * multiplier)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
         }
        // }
        else if(actuatorHeight.getAsDouble() > 50){
            m_drivetrainSubsystem.drive(
                // ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                resultX/1.5,
                resultY/1.5,
                m_rotationSupplier.getAsDouble()/1.5 * multiplier)
                // m_drivetrainSubsystem.getRotation2d()
                // )
            );
                }
            else{
                m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                        resultX,
                        resultY,
                        m_rotationSupplier.getAsDouble()/1.5 * multiplier)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
            }
        }
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
