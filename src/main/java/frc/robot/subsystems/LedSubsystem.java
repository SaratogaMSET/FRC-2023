package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{
    private Spark blinkin1;
    //private double val;
    public LedSubsystem(){
        blinkin1 = new Spark(0);
    }

    public void changeColor(double val){
        //val = (Math.random() * 2 - 1);
        this.blinkin1.set(val);
    }

    public void changeGreen(){
        changeColor(0.77);
    }

    public void changeRed(){
        changeColor(0.61);
    }

    public void changeIdle(){
        changeColor(-0.99);
    }
}