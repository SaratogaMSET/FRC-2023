package frc.robot.subsystems.Claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * No ClawConfig class to include limit switch and proximity sensor since those inputs don't affect simulation
 */
public class ClawIOVisualizer {
    private SingleJointedArmSim clawSim;
    public double appliedVoltage;
    public DCMotor motor;
    public AnalogInput proximitySensor;
    public ClawConfig config;
    public static Mechanism2d claw;
    public static MechanismLigament2d leftClaw;
    public static MechanismLigament2d rightClaw;
    public static MechanismRoot2d root;
    public static double initialAngle = 60;

    public ClawIOVisualizer() {
        claw = new Mechanism2d(2, 1);
        root = claw.getRoot("joint", 1, 0);
        leftClaw = root.append(new MechanismLigament2d("left_claw", 1, 90 + initialAngle));
        rightClaw = root.append(new MechanismLigament2d("right_claw", 1, 90 - initialAngle));
    }

    public void updateIntakeSim(double rotations){
        leftClaw.setAngle(90 + initialAngle - rotations * 360);
        rightClaw.setAngle(90 - initialAngle + rotations * 360);
        Logger.getInstance().recordOutput("Mechanisms/Claw", claw);
    }
}
