package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
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
    private final DoubleSupplier gunnerSupplier;
    private final DoubleSupplier armHeight;
    private final DoubleSupplier actuatorHeight;
    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;
    double translationalScaler = 0.5;
    double desiredAngle;
    int axis = 0;
    double lastRot;
    private PIDController controller = new PIDController(10, 0,0); //3.5
    double minArmValue = 0.3; 
    double maxArmValue = 1.18; 

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               DoubleSupplier gunnerSupplier,
                               DoubleSupplier armHeight,
                               DoubleSupplier actuatorHeight) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.gunnerSupplier = gunnerSupplier;
        this.armHeight = armHeight;
        this.actuatorHeight = actuatorHeight;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        m_translationXTrapezoidal = 0;
        m_translationYTrapezoidal = 0;
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
        controller.enableContinuousInput(90, 270); //90, 360
        axis = (int) m_drivetrainSubsystem.getRotation2d().getDegrees() / 180;
        if(m_drivetrainSubsystem.getRotation2d().getDegrees() - axis * 180 > 90) axis++ ;
        desiredAngle = (axis * 180.0) -90;
    }
    @Override
    public void execute() {
        double rotation = m_rotationSupplier.getAsDouble()/1.5;
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/6 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/6 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle); 

        double multiplier = Math.pow(Math.abs(m_rotationSupplier.getAsDouble()/7), 0.8);

        // double scaler = 1/(1- translationalScaler) * ( rotation* multiplier);

        double resultX = Math.cos(roboAngle) * magnitude;  //* scaler;
        double resultY = Math.sin(roboAngle) * magnitude; //* scaler;
        
        double armY = armHeight.getAsDouble();
    

        if(Math.abs(gunnerSupplier.getAsDouble()) < 2){
         if(armY > 0.45){
            lastRot = m_drivetrainSubsystem.getRotation2d().getDegrees(); //getPose().getRotation().getDegrees()
            axis = ( (int) lastRot) / 180 -1;
            if(lastRot - axis * 180 > 90) axis++ ;
            desiredAngle = (axis * 180.0) -90;
            double pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;

            m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                resultX/2.25,
                resultY/2.25,
                pidValue + rotation
                )
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
         }
        else if(actuatorHeight.getAsDouble() > 50){

            m_drivetrainSubsystem.drive(
                // ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                resultX/2,
                resultY/2,
                rotation * multiplier)
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
                        rotation * multiplier)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
            }
        }
            else{
                if(armY > 0.45 ){
                    lastRot =  m_drivetrainSubsystem.getRotation2d().getDegrees();
                    axis = ( (int) lastRot) / 180 -1;
                    if(lastRot - axis * 180 > 90) axis++ ;
                    desiredAngle =(axis * 180.0) -90;
                    double pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;
                        if(Math.abs(ArmNodeDictionary.ready_double_substation_y - armY) < 0.03 )
                            m_drivetrainSubsystem.drive(
                                    // ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(
                                    resultX/2.25,
                                    Math.copySign(0.5, gunnerSupplier.getAsDouble()),
                                    pidValue + rotation)
                                    // m_drivetrainSubsystem.getRotation2d()
                                    // )
                        );
                        else{
                            m_drivetrainSubsystem.drive(
                                    // ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(
                                    resultX/2.25,
                                    Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                                    pidValue + rotation)
                        );
                        }

                 }
                else if(actuatorHeight.getAsDouble() > 50){
                    m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                        resultX/2,
                        Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                        rotation * multiplier)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                    );
                        }
                    else{
                        m_drivetrainSubsystem.drive(
                                // ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(
                                resultX,
                                Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                                rotation * multiplier)
                                // m_drivetrainSubsystem.getRotation2d()
                                // )
                        );
                    }
            }
        }
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}