package frc.robot.TrajectoryLib.src.com.team254.lib.trajectory;

import frc.robot.TrajectoryLib.src.com.team254.lib.util.ChezyMath;

/**
 * A WaypointSequence is a sequence of Waypoints.  #whatdidyouexpect
 *
 * @author Art Kalb
 * @author Stephen Pinkerton
 * @author Jared341
 */
public class WaypointSequence {

  public static class Waypoint {

    public Waypoint(double x, double y, double theta) {
      this.x = x;
      this.y = y;
      this.theta = theta;
    }
    
    /**
     * @param pos pos[0] = x; pos[1] = y; pos[2] = theta
     */
    public Waypoint(double pos[]) {
        this.x = pos[0];
        this.y = pos[1];
        this.theta = pos[2];
      }
    
    public Waypoint(Waypoint tocopy) {
      this.x = tocopy.x;
      this.y = tocopy.y;
      this.theta = tocopy.theta;
    }

    public double x;
    public double y;
    public double theta;
  }

  Waypoint[] waypoints_;
  int num_waypoints_;

  public WaypointSequence(int max_size) {
    waypoints_ = new Waypoint[max_size];
  }

  public void addWaypoint(Waypoint w) {
    if (num_waypoints_ < waypoints_.length) {
      waypoints_[num_waypoints_] = w;
      ++num_waypoints_;
    }
  }

  public int getNumWaypoints() {
    return num_waypoints_;
  }

  public Waypoint getWaypoint(int index) {
    if (index >= 0 && index < getNumWaypoints()) {
      return waypoints_[index];
    } else {
      return null;
    }
  }
  
  public WaypointSequence invertY() {
    WaypointSequence inverted = new WaypointSequence(waypoints_.length);
    inverted.num_waypoints_ = num_waypoints_;
    for (int i = 0; i < num_waypoints_; ++i) {
      inverted.waypoints_[i] = waypoints_[i];
      inverted.waypoints_[i].y *= -1;
      inverted.waypoints_[i].theta = ChezyMath.boundAngle0to2PiRadians(
              2*Math.PI - inverted.waypoints_[i].theta);
    }
    
    return inverted;
  }
}