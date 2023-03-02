package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//import org.apache.commons.exec.ExecuteException;

//import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultLedCommand extends CommandBase {
    private VisionSubsystem m_VisionSubsystem;
    private LedSubsystem m_LedSubsystem;

    public DefaultLedCommand(LedSubsystem m_LedSubsystem, VisionSubsystem m_VisionSubsystem){
        this.m_VisionSubsystem = m_VisionSubsystem;
        this.m_LedSubsystem = m_LedSubsystem;
    }

    @Override
    public void execute() {
        if (m_VisionSubsystem.getPipeline() == Constants.VisionConstants.retroPipeline && m_VisionSubsystem.getOffsetTo2DOFBase()[0] < 1.0){
            m_LedSubsystem.changeGreen();
        } else {
            m_LedSubsystem.changeRed();
        }
    }

}
