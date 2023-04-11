package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmNodeDictionary;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class MoveWithClosest90 extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
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

    public MoveWithClosest90(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier armHeight,
        DoubleSupplier actuatorHeight,
        DoubleSupplier gunnerSupplier){
        
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.armHeight = armHeight;
        this.actuatorHeight = actuatorHeight;
        this.gunnerSupplier = gunnerSupplier;

        addRequirements(drivetrainSubsystem);  

    }
    @Override
    public void initialize(){
        controller.enableContinuousInput(-180, 180);
        axis = (int) m_drivetrainSubsystem.getRotation2d().getDegrees() / 90;
        if(m_drivetrainSubsystem.getRotation2d().getDegrees() - axis * 90 > 45) axis++ ;
        desiredAngle = axis * 90;
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/6 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/6 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle); 

        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;
        
        
        SmartDashboard.putNumber("m_translationXSupplier", m_translationXTrapezoidal);
        SmartDashboard.putNumber("m_translationYSupplier", m_translationYTrapezoidal);
        SmartDashboard.putNumber("Magnitude", magnitude);
        double armY = armHeight.getAsDouble();
        lastRot = m_drivetrainSubsystem.getRotation2d().getDegrees();
        axis = ( (int) lastRot) / 90 -1;
        if(lastRot - axis * 90 > 45) axis++ ;
        desiredAngle = axis * 90;
        double pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;

        if(Math.abs(gunnerSupplier.getAsDouble()) < 2){
         if(armY > 0.35){
            m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                resultX/2.25,
                resultY/2.25,
                pidValue
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
                pidValue)
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
                        pidValue)
                        // m_drivetrainSubsystem.getRotation2d()
                        // )
                );
            }
        }
            else{
                if(armY > 0.35 ){
                        if(Math.abs(ArmNodeDictionary.ready_double_substation_y - armY) < 0.03 )
                            m_drivetrainSubsystem.drive(
                                    // ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(
                                    resultX/2.25,
                                    Math.copySign(0.5, gunnerSupplier.getAsDouble()),
                                    pidValue)
                                    // m_drivetrainSubsystem.getRotation2d()
                                    // )
                        );
                        else{
                            m_drivetrainSubsystem.drive(
                                    // ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(
                                    resultX/2.25,
                                    Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                                    pidValue)
                        );
                        }

                 }
                else if(actuatorHeight.getAsDouble() > 50){
                    m_drivetrainSubsystem.drive(
                        // ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                        resultX/2,
                        Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                        pidValue)
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
                                pidValue)
                                // m_drivetrainSubsystem.getRotation2d()
                                // )
                        );
                    }
            }
        }

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }

    @Override
    public boolean isFinished(){
        // if(Math.abs(drivetrain.getRotation2d().getDegrees()-desiredAngle)<3) return true;
        return false;
    }
}
