// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggableChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.drivers.LazyTalonFX;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * <b>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     * </b>
     * </p>
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI); // 4.968230455
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0); // 11.3450651

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics,
            new Rotation2d(0), getPosition());

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.

    public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    public double offset = 0;

    // These are our modules. We initialize them in the constructor.
    public final SwerveModule m_frontLeftModule;
    public final SwerveModule m_frontRightModule;
    public final SwerveModule m_backLeftModule;
    public final SwerveModule m_backRightModule;

    public final double frontLeftEncoderOffset = Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET;
    public final double backLeftEncoderOffset = Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET;
    public final double frontRightEncoderOffset = Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
    public final double backRightEncoderOffset = Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR;

    private SwerveModuleState[] currentState = new SwerveModuleState[4];
    private SwerveModuleState[] previousState = new SwerveModuleState[4];

    private LoggableChassisSpeeds SimvSpeeds = new LoggableChassisSpeeds("/SwerveDriveSubsystem/Velocity",
            new ChassisSpeeds(0.0, 0.0, 0.0));
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private TalonFX m_frontLeftSteer;
    private TalonFX m_frontRightSteer;
    private TalonFX m_backLeftSteer;
    private TalonFX m_backRightSteer;

    private LazyTalonFX m_frontLeftDrive;
    private LazyTalonFX m_frontRightDrive;
    private LazyTalonFX m_backLeftDrive;
    private LazyTalonFX m_backRightDrive;

    public DrivetrainSubsystem() {

        m_frontLeftDrive = new LazyTalonFX(Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR);
        m_frontRightDrive = new LazyTalonFX(Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        m_backLeftDrive = new LazyTalonFX(Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR);
        m_backRightDrive = new LazyTalonFX(Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR);
        m_frontLeftSteer = new TalonFX(Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR);
        m_frontRightSteer = new TalonFX(Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR);
        m_backLeftSteer = new TalonFX(Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR);
        m_backRightSteer = new TalonFX(Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR);

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                // .withSize(2, 4)
                // .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET);

        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                // .withSize(2, 4)
                // .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                // .withSize(2, 4)
                // .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                // .withSize(2, 4)
                // .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        offset = m_navx.getFusedHeading();
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        // new SequentialCommandGroup(
        // new WaitCommand(1),
        // new InstantCommand(() -> drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        // new InstantCommand(() -> resetOdometry(new Pose2d()))
        // ).schedule();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.toDegrees(getNavHeading()));
    }

    public Rotation2d getAngle() {
        double angle = -m_navx.getYaw() + 180;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return Rotation2d.fromDegrees(angle);
    }

    public double getNavHeading() {

        double angle = m_navx.getFusedHeading() - offset;
        angle = angle % 360;
        return Math.toRadians(angle + 90);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getPosition(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d startRotation) {
        odometer.resetPosition(startRotation, getPosition(), pose);
    }

    public SwerveModulePosition[] getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(m_frontLeftDrive.getSelectedSensorPosition(),
                        new Rotation2d(m_frontLeftSteer.getSelectedSensorPosition() - frontLeftEncoderOffset)),
                new SwerveModulePosition(m_frontRightDrive.getSelectedSensorPosition(),
                        new Rotation2d(m_frontRightSteer.getSelectedSensorPosition() - frontRightEncoderOffset)),
                new SwerveModulePosition(m_backLeftDrive.getSelectedSensorPosition(),
                        new Rotation2d(m_backLeftSteer.getSelectedSensorPosition() - backLeftEncoderOffset)),
                new SwerveModulePosition(m_backRightDrive.getSelectedSensorPosition(),
                        new Rotation2d(m_backRightSteer.getSelectedSensorPosition() - backRightEncoderOffset))
        };
        return swerveModulePositions;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        double IpX = Units.inchesToMeters(2);
        double IpY = Units.inchesToMeters(-5.4);
        double IpD = Math.hypot(IpY, IpX);
        double theta = Math.atan2(IpY, IpX);
        double dT = 0.02;
        double omega = chassisSpeeds.omegaRadiansPerSecond;
        double deltaTheta = dT * omega;
        chassisSpeeds.vxMetersPerSecond += IpD * omega
                * (Math.cos(theta) * Math.cos(deltaTheta) - Math.sin(theta) * Math.sin(deltaTheta));
        chassisSpeeds.vyMetersPerSecond += IpD * omega
                * (Math.sin(theta) * Math.cos(deltaTheta) + Math.cos(theta) * Math.sin(deltaTheta));
        SimvSpeeds.set(chassisSpeeds);
        m_chassisSpeeds = chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] setState) {
        SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(setState[0],
                new Rotation2d(m_frontLeftModule.getSteerAngle()));
        SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(setState[1],
                new Rotation2d(m_frontRightModule.getSteerAngle()));
        SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(setState[2],
                new Rotation2d(m_backLeftModule.getSteerAngle()));
        SwerveModuleState backRightOptimized = SwerveModuleState.optimize(setState[3],
                new Rotation2d(m_backRightModule.getSteerAngle()));

        m_frontLeftModule.set(frontLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                frontLeftOptimized.angle.getRadians());
        m_frontRightModule.set(frontRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                frontRightOptimized.angle.getRadians());
        m_backLeftModule.set(backLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                backLeftOptimized.angle.getRadians());
        m_backRightModule.set(backRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                backRightOptimized.angle.getRadians());

        // DataLogManager.start();

        // // Set up custom log entries
        // DataLog log = DataLogManager.getLog();
        // myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
        // myDoubleLog = new DoubleLogEntry(log, "/my/double");
        // myStringLog = new StringLogEntry(log, "/my/string");

        odometer.update(
                getRotation2d(),
                getPosition());
    }

    public SwerveModuleState[] getCurrentStates() {
        return currentState;
    }

    @Override
    public void periodic() {
        previousState = currentState;
        double[] SpeedsArray = new double[] { m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond,
                m_chassisSpeeds.omegaRadiansPerSecond };
        SmartDashboard.putNumberArray("targetChassisSpeeds", SpeedsArray); // this is getting no theta velocity
        currentState = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(currentState, MAX_VELOCITY_METERS_PER_SECOND);

        if (previousState[0] == null || previousState[1] == null || previousState[2] == null
                || previousState[3] == null) {
            previousState = currentState;
        }

        double joystickCenterState = 0;
        boolean joystickCentered = true;
        for (int i = 0; i < currentState.length && joystickCentered; i++) {
            if (Math.abs(currentState[i].angle.getRadians() - joystickCenterState) > 0.001)
                joystickCentered = false;
        }

        if (joystickCentered) {
            currentState[0].angle = previousState[0].angle;
            currentState[1].angle = previousState[1].angle;
            currentState[2].angle = previousState[2].angle;
            currentState[3].angle = previousState[3].angle;
        }

        setModuleStates(currentState);

        SmartDashboard.putNumber("Forward Left", m_frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Forward Right", m_frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("Back Left", m_backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Back Right", m_backRightModule.getSteerAngle());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
        SmartDashboard.putNumber("Chassis Angular Speed", m_chassisSpeeds.omegaRadiansPerSecond);
    }
}