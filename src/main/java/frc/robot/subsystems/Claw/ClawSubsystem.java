// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{

    /*  private NetworkTable table = new NetworkTable(NetworkTable.getInstance(), "/Claw") ;
     * 
    */

    private double[] proximityBuffer = new double[3];
    private int bufferIndex = 0;
    private double cubeBlueTheshold = 0.22;
    private Color currentColor;
    public boolean startedAutoClose = false;
    public ColorSensorV3 colorSensor;
    public CANSparkMax motor = new CANSparkMax(ClawConstants.INTAKE_MOTOR, MotorType.kBrushless);
    public RelativeEncoder encoder = motor.getEncoder();
    String print = "Claw/"; 
    private boolean hasZeroedEncoder = false;
    double minVelocity = 0.1;
    double maxVelocity = 0.85;
    double curve = 0.45; //0.45

    /* TODO make state enums */
    private boolean acquired = false;
    private boolean flash = false;
    public double time;

    public static enum GamePiece {
        Cone,
        Cube,
        None
    }

    public ClawSubsystem() {
        motor.setIdleMode(IdleMode.kBrake);
        colorSensor = new ColorSensorV3(Port.kOnboard);
        motor.setSmartCurrentLimit(30);
    }

    /* true = we got  a game piece. False = we don't */
    public boolean isGamepieceAcquired(){
        return acquired;
    }

    public double getTime(){
        return time;
    }

    public boolean getFlash(){
        return flash;
    }
    public void setFlash(boolean x){
        flash = x;
    }

    public void setBrakeMode(){
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        motor.setIdleMode(IdleMode.kCoast);
    }


    public boolean hasAcquiredGamePiece(){
        if(isGamepieceInRange() &&
            getGamePieceType() == GamePiece.Cone &&
            getConeTolerance()) return true;

        else if(isGamepieceInRange() &&
            getGamePieceType() == GamePiece.Cube &&
            getCubeTolerance()){
    
            return true;
        }
        return false; 
    }
    public boolean getConeTolerance(){
       return encoder.getPosition() >= ClawConstants.CONE_MEDIUM_BOUND - 4;
    }
    public boolean getCubeTolerance(){
        return encoder.getPosition() >= ClawConstants.CUBE_MEDIUM_BOUND - 1;
     }
    public void setIdle() {
        motor.set(0.0);
        startedAutoClose = false;
    }

    private double getProximityValue() {
        proximityBuffer[bufferIndex] = colorSensor.getProximity();
        bufferIndex++;
        bufferIndex = bufferIndex % proximityBuffer.length;

        double sum = 0;
        for (double x : proximityBuffer)
            sum += x;
        return sum / proximityBuffer.length;
    }

    public boolean isGamepieceInRange() {
        return getProximityValue() > 90; //65
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }  

    public void openClaw() {
        updateClawTelemetry();
        double openVelocity = 0.0;
        if(isClawFullyOpen()) {
            openVelocity = 0.0; 
            hasZeroedEncoder = true; // NEW
            resetEncoder();
        }
        else{   

            //NEW:
           
            if(encoder.getPosition() < 25
            ){
                openVelocity = -0.25;
            }
            else{
                openVelocity = -0.81;
            }
            //end new code
        }
        motor.set(openVelocity);
        acquired = false;
        flash = false;
    }


    public void autoCloseClaw() {
        if(isGamepieceInRange()) {
            // if (isClawFullyOpen() || startedAutoClose) {
                // Either the claw is fully open or we had previously started auto close.
                startedAutoClose = true;
                double encoderPosition = encoder.getPosition();
                if (getGamePieceType() == GamePiece.Cube) {
                    if(encoderPosition >= ClawConstants.CUBE_MEDIUM_BOUND - 1) {
                        motor.set(0.0);
                        acquired = true;
                        flash = true;
                        // time = Timer.getFPGATimestamp();
                    } else {
                        double encoderPositionRatio = (0.0-encoder.getPosition()) / (0.0-ClawConstants.CONE_MEDIUM_BOUND);
                        encoderPositionRatio = 1 - Math.max(0.0, Math.min(encoderPositionRatio, 1.0));
                        encoderPositionRatio = Math.pow(encoderPositionRatio, curve);
                        encoderPositionRatio = Math.max(minVelocity, Math.min(encoderPositionRatio, maxVelocity));
                        motor.set(encoderPositionRatio);
                    }
                
                } else if (getGamePieceType() == GamePiece.Cone) {
                    if (encoderPosition >= ClawConstants.CONE_MEDIUM_BOUND - 1) {
                        motor.set(0.0);
                        acquired = true;
                        flash = true;
                        // time = Timer.getFPGATimestamp();
                    } else {
                        double encoderPositionRatio = (0.0-encoder.getPosition()) / (0.0-ClawConstants.CONE_MEDIUM_BOUND);
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

    public void manualCloseClaw() {
        if (isClawFullyClosed()) {
            motor.set(0.0);
        } else {
            double encoderPositionRatio = (0.0-encoder.getPosition()) / (0.0-ClawConstants.CONE_MEDIUM_BOUND);
            encoderPositionRatio = 1 - Math.max(0.0, Math.min(encoderPositionRatio, 1.0));
            encoderPositionRatio = Math.pow(encoderPositionRatio, curve);
            encoderPositionRatio = Math.max(minVelocity, Math.min(encoderPositionRatio, maxVelocity));
            motor.set(encoderPositionRatio);
        }
    }

    public GamePiece getGamePieceType() {
        currentColor = colorSensor.getColor();
        if (currentColor == null) {
            return GamePiece.None;
        }

        double magnitude = currentColor.blue + currentColor.red + currentColor.green;
        if (currentColor.blue / magnitude > cubeBlueTheshold)
            return GamePiece.Cube;
        else
            return GamePiece.Cone;
    }

    private boolean isClawFullyClosed() {
        return motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public boolean isClawFullyOpen() {
        return motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();
    }

    public void updateClawTelemetry() {
        SmartDashboard.putNumber("ClawPos", encoder.getPosition());
        // SmartDashboard.putBoolean("Limit Switch", getHallEffect());
        // SmartDashboard.putBoolean("Is Object in Range", isGamepieceInRange());
        SmartDashboard.putNumber("Proximity Value", getProximityValue());
        // SmartDashboard.putBoolean("Detecting", objectInRange());
        // SmartDashboard.putNumber("Red value", colorSensor.getRed());
        // SmartDashboard.putNumber("Green value", colorSensor.getGreen());
        // SmartDashboard.putNumber("Blue value", colorSensor.getBlue());
        // SmartDashboard.putBoolean("Hall Effect", getHallEffect());
        SmartDashboard.putString("Object", getGamePieceType().toString());
        // SmartDashboard.putBoolean("Object in range", objectInRange());
        SmartDashboard.putBoolean("Is Claw Fully Open", isClawFullyOpen());
        SmartDashboard.putBoolean("Is Claw Fully Closed", isClawFullyClosed());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateClawTelemetry();
        // if (Timer.getFPGATimestamp() - time > 2) flash = false;
        // SmartDashboard.putNumber("TIME IS DOOOMED", time);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}