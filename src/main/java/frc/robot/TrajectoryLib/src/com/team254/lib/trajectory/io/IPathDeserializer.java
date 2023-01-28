package frc.robot.TrajectoryLib.src.com.team254.lib.trajectory.io;

import frc.robot.TrajectoryLib.src.com.team254.lib.trajectory.Path;

/**
 * Interface for methods that deserializes a Path or Trajectory.
 * 
 * @author Jared341
 */
public interface IPathDeserializer {
  
  public Path deserialize(String serialized);
}
