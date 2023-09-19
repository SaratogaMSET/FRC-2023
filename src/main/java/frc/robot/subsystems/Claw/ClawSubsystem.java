// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {

    private double[] proximityBuffer = new double[3]; // Create an array to keep track of three most recent proximity
                                                      // sensor readings
    private int bufferIndex = 0; // Moving index to update proximity sensor buffer
    private double cubeBlueTheshold = 0.22; // Minimum blue value to differentiate cube and cone
    private Color currentColor;
    public boolean startedAutoClose = false;
    public ColorSensorV3 colorSensor;
    public CANSparkMax motor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);
    public RelativeEncoder encoder = motor.getEncoder();
    String print = "Claw/"; // Folder header for diagnostics
    double minVelocity = 0.1; // Minimum velocity needed to overcome static friction
    double maxVelocity = 0.85; // Maximum velocity
    double curve = 0.45; // Root curve exponent for velocity

    /* TODO make state enums */
    private boolean acquired = false; // Set initial state of object acquired to false
    private boolean flash = false; // set initial state of LED's to not signify object acquired
    public double time;

    public static enum GamePiece {
        Cone,
        Cube,
        None
    }

    public ClawSubsystem() {
        motor.setIdleMode(IdleMode.kBrake); // Set motor to brake mode
        colorSensor = new ColorSensorV3(Port.kOnboard); // kmxp
        /*
         * Test was done without the eboard cover on
         * kOnboard: Cube in the center : 63 +-3
         * : Cube High touching the color sensor: 125 +- 3
         * kMxp: Cube in the center : 60 - 68
         * : Cube High touching the color sensor: 122 +- 3
         */
        motor.setSmartCurrentLimit(30); // Set motor current limit to prevent motor damage
    }

    /**
     * Set motor to brake when speed is set to zero
     */
    public void setBrakeMode() {
        motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Set motor to coast when speed is set to zero
     */
    public void setCoastMode() {
        motor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Check if game piece is within proximity sensor threshold and encoder threshold
     * 
     * @return true if game piece has been acquired, false if not
     */
    public boolean hasAcquiredGamePiece() {
        if (isGamepieceInRange() &&
                getGamePieceType() == GamePiece.Cone && // If color sensor detects a cone and encoder measurement is within cone encoder bound, cone has been acquired
                closedToConeTolerance())
            return true;

        else if (isGamepieceInRange() &&
                getGamePieceType() == GamePiece.Cube && // If color sensor detects a cube and encoder measurement is within cube encoder bound, cube has been acquired
                closedToCubeTolerance()) {

            return true;
        }
        return false;
    }

    /**
     * Check if claw has closed enough to have acquired a cone
     * 
     * @return true if claw is closed past cone encoder tolerance, false if not
     */
    public boolean closedToConeTolerance() {
        return encoder.getPosition() >= ClawConstants.CONE_MEDIUM_BOUND - 4;  // If within 4 rotations of cone encoder tolerance - cone collapses a lot
    }

    /**
     * Check if claw has closed enough to have acquired a cube
     * 
     * @return true if claw is closed past cube encoder tolerance, false if not
     */
    public boolean closedToCubeTolerance() {
        return encoder.getPosition() >= ClawConstants.CUBE_MEDIUM_BOUND - 1;    // If within 1 rotation of cube encoder tolerance
    }

    /**
     * Stop claw motor and disable autoclose
     */
    public void setIdle() {
        motor.set(0.0);
        startedAutoClose = false;   // not requesting autoclose
    }

    /**
     * Get value of proximity sensor in native units and update proximity buffer. 
     * Average proximity buffer values to cancel out white noise/momentary changes
     * 
     * @return averaged proximity buffer value
     */
    private double getProximityValue() {
        proximityBuffer[bufferIndex] = colorSensor.getProximity(); // Set oldest value in proximity buffer to current proximity value
        bufferIndex++; // Iterate buffer index to index of next oldest value
        bufferIndex = bufferIndex % proximityBuffer.length; // Wrap around buffer index (if at end of buffer, next index is 0 again)

        double sum = 0;
        for (double x : proximityBuffer) // Sum all values in proximity buffer
            sum += x;
        return sum / proximityBuffer.length; // Average all values in proximity buffer
    }

    /**
     * Check if game piece is within distance to tip of fully closed claw -- 90 native units of the proximity sensor, which is a tested value (sensor values decrease as object is futher away)
     * @return true if object is within range
     */
    public boolean isGamepieceInRange() {
        return getProximityValue() > 90;
    }

    /**
     * Reset current encoder position to 0 ticks
     */
    private void resetEncoder() {
        encoder.setPosition(0);
    }

    /**
     * Fully open the claw with adjusting speed to avoid breakage
     */
    public void openClaw() {
        // updateClawTelemetry();
        double openVelocity = 0.0;
        if (isClawFullyOpen()) {    // Check if "open" limit switch on lead screw has been triggered
            openVelocity = 0.0;
            resetEncoder();
        } else {
            if (encoder.getPosition() < 25) {   // If within 25 encoder rotations (tested metric) of fully open, slow down to 25% speed
                openVelocity = -0.25;
            } else {    // Otherwise, open at 81% speed
                openVelocity = -0.81;
            }
        }
        motor.set(openVelocity);
        acquired = false;
        flash = false;
    }

    /**
     * Close claw autonomously according to color sensor detection of object
     */
    public void autoCloseClaw() { 
        if (isGamepieceInRange()) {  // check if game piece is within distance of tip of closed claw
            // Either the claw is fully open or we had previously started auto close.
            startedAutoClose = true;
            double encoderPosition = encoder.getPosition();
            if (getGamePieceType() == GamePiece.Cube) { // Closing sequence for cube
                if (encoderPosition >= ClawConstants.CUBE_MEDIUM_BOUND - 1) {   // If within 1 rotation of cube encoder threshold, stop
                    motor.set(0.0);
                } else {    // Slow down exponentially as approaching target (really only slows down near end of motion) like: *****-_
                    double encoderPositionRatio = (0.0 - encoder.getPosition()) // currentEncoderPos / targetEncoderPos              *.
                            / (0.0 - ClawConstants.CONE_MEDIUM_BOUND);
                    encoderPositionRatio = 1 - Math.max(0.0, Math.min(encoderPositionRatio, 1.0)); // If [0 < encoderPosRatio < 1] then subtract from 1; otherwise, set ratio = 0
                    encoderPositionRatio = Math.pow(encoderPositionRatio, curve); // set to exponential curve
                    encoderPositionRatio = Math.max(minVelocity, Math.min(encoderPositionRatio, maxVelocity)); // ensure within max and min motor velocity bounds
                    motor.set(encoderPositionRatio);
                }

            } else if (getGamePieceType() == GamePiece.Cone) {  // Closing sequence for cone
                if (encoderPosition >= ClawConstants.CONE_MEDIUM_BOUND - 1) {   // If within 1 motor rotation of cone encoder threshold, stop
                    motor.set(0.0);
                } else {
                    double encoderPositionRatio = (0.0 - encoder.getPosition())  // Same motion profile as for Cube closing sequence
                            / (0.0 - ClawConstants.CONE_MEDIUM_BOUND);
                    encoderPositionRatio = 1 - Math.max(0.0, Math.min(encoderPositionRatio, 1.0));
                    encoderPositionRatio = Math.pow(encoderPositionRatio, curve);
                    encoderPositionRatio = Math.max(minVelocity, Math.min(encoderPositionRatio, maxVelocity));
                    motor.set(encoderPositionRatio);
                }
            } else {
                motor.set(0.0);
            }
        }
        // }
    }

    /**
     * Close claw without condition of game piece being in range (manual override version)
     */
    public void manualCloseClaw() {
        if (isClawFullyClosed()) {
            motor.set(0.0);
        } else {    // Same motion profile as in autoCloseClaw()
            double encoderPositionRatio = (0.0 - encoder.getPosition()) / (0.0 - ClawConstants.CONE_MEDIUM_BOUND);
            encoderPositionRatio = 1 - Math.max(0.0, Math.min(encoderPositionRatio, 1.0));
            encoderPositionRatio = Math.pow(encoderPositionRatio, curve);
            encoderPositionRatio = Math.max(minVelocity, Math.min(encoderPositionRatio, maxVelocity));
            motor.set(encoderPositionRatio);
        }
    }

    /**
     * Use color sensor to classify game piece, assuming proximity sensor detects object
     * @return GamePiece.None, GamePiece.Cube or GamePiece.Cone
     */
    public GamePiece getGamePieceType() {
        currentColor = colorSensor.getColor();
        if (currentColor == null) { // Prevent current color attributes from being called if colorSensor is not initialized/failing
            return GamePiece.None;
        }

        double magnitude = currentColor.blue + currentColor.red + currentColor.green;   // calculate brightness in snapshot
        if (currentColor.blue / magnitude > cubeBlueTheshold)   // check if blue value of object is above tested cube blue threshold, adjust for brightness by dividing by brightness
            return GamePiece.Cube;
        else
            return GamePiece.Cone;  // Otherwise default to cone
    }

    /**
     * Check if claw is fully closed
     * @return value of closed limit switch (triggered when motor reaches limit moving forward)
     */
    private boolean isClawFullyClosed() {
        return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    /**
     * Check if claw is fully open
     * @return value of opened limit switch (triggered when motor reaches limit moving in reverse)
     */
    public boolean isClawFullyOpen() {
        return motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    /**
     * Print Claw states to SmartDashboard
     */
    public void updateClawTelemetry() {
        SmartDashboard.putNumber("ClawPos", encoder.getPosition());
        // SmartDashboard.putBoolean("Is Object in Range", isGamepieceInRange());
        // SmartDashboard.putNumber("Proximity Value", getProximityValue());
        // SmartDashboard.putBoolean("Detecting", isGamepieceInRange());
        // SmartDashboard.putNumber("Red value", colorSensor.getRed());
        // SmartDashboard.putNumber("Green value", colorSensor.getGreen());
        // SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
        SmartDashboard.putString("Object", getGamePieceType().toString());
        // SmartDashboard.putBoolean("Object in range", objectInRange());
        SmartDashboard.putBoolean("Is Claw Fully Open", isClawFullyOpen());
        SmartDashboard.putBoolean("Is Claw Fully Closed", isClawFullyClosed());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Opening Claw", motor.get());  // Print claw speed from [-1.0, 1.0]
        updateClawTelemetry(); // Print other claw states to SmartDashboard
        // Logger.getInstance().recordOutput("Proximity Value", getProximityValue());
    }
    
    // This method will be called once per scheduler run during simulation
    @Override
    public void simulationPeriodic() {
    }
}