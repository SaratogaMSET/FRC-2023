package frc.robot.util.wrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveOdomMeasurement {
    private final Rotation2d gyroAngle;
    private final SwerveModuleState[] moduleStates;

    public SwerveOdomMeasurement(
        Rotation2d gyroAngle,
        SwerveModuleState... moduleStates
    ) {
        this.gyroAngle = gyroAngle;
        this.moduleStates = moduleStates;
    }

    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }

    public SwerveModuleState[] getModuleStates() {
        return moduleStates;
    }
}
