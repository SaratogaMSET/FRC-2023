package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmNodeDictionary;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
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
    double speedLimit = Drivetrain.speedLimit;

    private enum Speed {
        NORMAL,
        SLOW,
        SLOWEST
    }

    SlewRateLimiter translationX;
    SlewRateLimiter translationY;
    SlewRateLimiter rotationLimiter;

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
                               DoubleSupplier actuatorHeight
                                ) {
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
        translationX = new SlewRateLimiter(3.6);
        translationY = new SlewRateLimiter(3.6);
        rotationLimiter = new SlewRateLimiter(5.05);

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
        controller.enableContinuousInput(90, 270); //90, 360
        axis = (int) m_drivetrainSubsystem.getRotation2d().getDegrees() / 180;
        if(m_drivetrainSubsystem.getRotation2d().getDegrees() - axis * 180 > 90) axis++ ;
        desiredAngle = (axis * 180.0) -90;
    }
    @Override
    public void execute() {
        double armY = armHeight.getAsDouble();

        axis = (int) m_drivetrainSubsystem.getRotation2d().getDegrees() / 180;
        if(m_drivetrainSubsystem.getRotation2d().getDegrees() - axis * 180 > 90) axis++ ;
        desiredAngle = (axis * 180.0) -90;
        double pidValue = controller.calculate(lastRot, desiredAngle) * Math.PI/180;

        Speed speed =
            armY > 0.35
                ? Speed.SLOWEST
                : actuatorHeight.getAsDouble() > 50 ? Speed.SLOW : Speed.NORMAL;

        switch(speed){
            case NORMAL:
                speedLimit = Drivetrain.speedLimit;
            break;
            case SLOW:
                speedLimit = Drivetrain.slowSpeedLimit;
            break;
            case SLOWEST:
                speedLimit = Drivetrain.slowestSpeedLimit;
            break;
        default:
                speedLimit = Drivetrain.speedLimit;
            break;
        }

        double rotation = m_rotationSupplier.getAsDouble()/1.5;
        SmartDashboard.putNumber("desiredAngle", desiredAngle);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle); 

        double magnitude = Math.hypot(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());

        double multiplier = Math.pow(Math.abs(m_rotationSupplier.getAsDouble()/7), 0.8);

        // double scaler = 1/(1- translationalScaler) * ( rotation* multiplier);

        double resultX = Math.cos(roboAngle) * magnitude;  //* scaler;
        double resultY = Math.sin(roboAngle * magnitude); //* scaler;
        if(armY <= 0.35){
            if(gunnerSupplier.getAsDouble()<2){
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    translationY.calculate(resultY * speedLimit),
                    rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
            else if(gunnerSupplier.getAsDouble()>2){
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                    rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
            else{
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    translationY.calculate(resultY * speedLimit),
                    rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
    }
        else if(armY >=0.35){
            if(gunnerSupplier.getAsDouble()<2){
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    translationY.calculate(resultY * speedLimit),
                    pidValue + rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
            else if(gunnerSupplier.getAsDouble()>2){
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    Math.copySign(0.35, gunnerSupplier.getAsDouble()),
                    pidValue + rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
            else{
                m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                    translationX.calculate(resultX * speedLimit),
                    translationY.calculate(resultY * speedLimit),
                    pidValue + rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                    )
                );
            }
        }
        else{
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                translationX.calculate(resultX * speedLimit),
                translationY.calculate(resultY * speedLimit),
                rotationLimiter.calculate(rotation * multiplier * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                )
            );
        }

        }
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}