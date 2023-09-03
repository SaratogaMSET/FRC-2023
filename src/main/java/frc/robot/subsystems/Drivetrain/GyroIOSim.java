package frc.robot.subsystems.Drivetrain;

@Deprecated
/** Still getting used to our sim software**/
public class GyroIOSim implements GyroIO{

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.yawPositionRad = 0;
        inputs.yawVelocityRadPerSec = 0;
        inputs.pitchPositionRad = 0;
        inputs.rollPositionRad = 0;
    }
}