package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

public class TimestampedSwerveOdometry {
    private final SwerveDriveKinematics kinematics;
    private Pose2d poseMeters;
    private double prevTimeSeconds = -1;

    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;

    public TimestampedSwerveOdometry(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        Pose2d initialPose
    ) {
        this.kinematics = kinematics;
        this.poseMeters = initialPose;
        gyroOffset = poseMeters.getRotation().minus(gyroAngle);
        previousAngle = initialPose.getRotation();
    }

    public TimestampedSwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
        poseMeters = pose;
        previousAngle = pose.getRotation();
        gyroOffset = pose.getRotation().minus(gyroAngle);
    }

    public Pose2d getPoseMeters() {
        return poseMeters;
    }

    public Pose2d updateWithTime(
        double prevTimeSeconds,
        double currentTimeSeconds,
        Rotation2d gyroAngle,
        SwerveModuleState... moduleStates
    ) {
        double period = prevTimeSeconds >= 0 ? currentTimeSeconds - prevTimeSeconds : 0d;
        this.prevTimeSeconds = prevTimeSeconds;

        var angle = new Rotation2d(gyroAngle.getRadians() + gyroOffset.getRadians());

        var chassisState = kinematics.toChassisSpeeds(moduleStates);
        var newPose = poseMeters.exp(
            new Twist2d(
                chassisState.vxMetersPerSecond * period,
                chassisState.vyMetersPerSecond * period,
                angle.minus(previousAngle).getRadians()
            )
        );

        previousAngle = angle;
        poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return poseMeters;
    }

    public Pose2d updateWithTime(
        double currentTimeSeconds,
        Rotation2d gyroAngle,
        SwerveModuleState... moduleStates
    ) {
        double period = prevTimeSeconds >= 0 ? currentTimeSeconds - prevTimeSeconds : 0.0;
        prevTimeSeconds = currentTimeSeconds;

        var angle = new Rotation2d(gyroAngle.getRadians() + gyroOffset.getRadians());

        var chassisState = kinematics.toChassisSpeeds(moduleStates);
        var newPose = poseMeters.exp(
            new Twist2d(
                chassisState.vxMetersPerSecond * period,
                chassisState.vyMetersPerSecond * period,
                angle.minus(previousAngle).getRadians()
            )
        );

        previousAngle = angle;
        poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return poseMeters;
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
        return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, moduleStates);
    }
}
