package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SwitchPipeline extends CommandBase{
    private VisionSubsystem m_vision;
    private int pipelineN = 0; // pipeline number

    public SwitchPipeline(VisionSubsystem m_vision, int pipelineN){
        this.m_vision = m_vision;
        this.pipelineN = pipelineN;
    }

    @Override
    public void execute() {
        m_vision.switchPipeline(pipelineN);
    }
}
