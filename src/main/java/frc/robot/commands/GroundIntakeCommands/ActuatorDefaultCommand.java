// package frc.robot.commands.GroundIntakeCommands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.GroundIntake.ActuatorSubsystem;

// public class ActuatorDefaultCommand extends CommandBase{
    
//     DoubleSupplier x;
//     DoubleSupplier y;
//     ActuatorSubsystem actuator;
//     public ActuatorDefaultCommand(ActuatorSubsystem actuatorSubsystem,DoubleSupplier armEEPositionX, DoubleSupplier armEEPositionY){
//         actuator = actuatorSubsystem;
//         x = armEEPositionX;
//         y = armEEPositionY;
//         addRequirements(actuatorSubsystem);
//     }


//     @Override
//     public void execute(){
//         if(actuator.detect_collision(new double[x.getAsDouble(), y.getAsDouble()], 0)){

//         }
//     }
// }
