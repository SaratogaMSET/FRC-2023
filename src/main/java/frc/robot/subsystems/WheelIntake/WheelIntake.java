package frc.robot.subsystems.WheelIntake;

import java.util.Currency;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelIntakeConstants;
import frc.robot.util.SuppliedWaitCommand;

public class WheelIntake extends SubsystemBase{
    public enum Gamepiece{
        CONE,
        CUBE
    }
    Gamepiece currentGamepiece = Gamepiece.CONE; //TODO: SYNC WITH LEDS ON THE FULL ROBOT CUS OWEN 
    public WPI_TalonFX intake = new WPI_TalonFX(WheelIntakeConstants.IntakeId, "649-Hammerhead-CANivore"); 
    public double[] buffer = new double[5]; 
    private int bufferIndex = 0;
    private double intakeVolts = 12;    
    public WheelIntake(){
        //supply configurations
        // intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 14, 0.1));
        intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,15, 40, 2));
        intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10,0.0));
        intake.setNeutralMode(NeutralMode.Brake);
        intake.enableVoltageCompensation(true);
        intake.configNeutralDeadband(0.0);
    }
    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public double getVoltage(){
       return intake.getMotorOutputVoltage();
    }
    public void set(double set){
        intake.set(set);
    }
    public double getVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(falconToRPM(intake.getSelectedSensorVelocity(), WheelIntakeConstants.reduction));
    }
    public double getStatorCurrent(){
        return intake.getStatorCurrent();
    }
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }
    public void holdGamePiece(){
        intake.setVoltage(WheelIntakeConstants.holdVolts);
        // configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 4, 4, 0.0));
    }
    public void configStatorCurrentLimit(StatorCurrentLimitConfiguration limit){
        intake.configStatorCurrentLimit(limit);
    }
    public double buffer(){
        buffer[bufferIndex] = getStatorCurrent(); // Set oldest value in proximity buffer to current proximity value
        bufferIndex++; // Iterate buffer index to index of next oldest value
        bufferIndex = bufferIndex % buffer.length; // Wrap around buffer index (if at end of buffer, next index is 0 again)

        double sum = 0;
        for (double x : buffer) // Sum all values in proximity buffer
            sum += x;
        return sum / buffer.length;
    }
    public Command intakeCommand() {
        return run(() -> setVoltage(intakeVolts))
            .raceWith(
                new SuppliedWaitCommand(() -> WheelIntakeConstants.intakeVelocityWaitStart)
                    .andThen(
                        Commands.waitUntil(() -> getVelocity() < WheelIntakeConstants.stopVelocity),
                        new SuppliedWaitCommand(() -> WheelIntakeConstants.intakeVelocityWaitStop)))
            .finallyDo((interrupted) -> setVoltage(WheelIntakeConstants.holdVolts));
      }
      public boolean hasAcquiredGamePiece(){
        return  getVelocity() <= WheelIntakeConstants.stopVelocity && (12* Math.abs(intake.get())) <= WheelIntakeConstants.holdVolts;
      }
      public boolean isGamePieceAcquired(){
        return intake.getMotorOutputVoltage() >= WheelIntakeConstants.holdVolts && getVelocity() <= WheelIntakeConstants.stopVelocity;
      }
      
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Current Output Current", getStatorCurrent());
        SmartDashboard.putNumber("Current Motor Output Voltage", getVoltage());
        SmartDashboard.putNumber("Buffer value", buffer());
        SmartDashboard.putNumber("Velocity", getVelocity());
        // if(!isConeAcquired() && !isCubeAcquired()){
        //     intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 5, 40, 0.1));
        // }
    }
}
