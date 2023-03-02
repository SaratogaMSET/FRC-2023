# TrajectoryLib-254-adapted
This is an adaptation of FRC 254's TrajectoryLib. They created this code, I just adapted it for a WPI Command-Based robot
This Uses FRC254's Spline Path Generating Code and FeedFoward Path Following. HUGE Thanks to them!!!:<br/>
https://github.com/Team254/TrajectoryLib
<br/><br/><br/>

## How To Use (as of 02/25/2017)

### Step 1:
In a new scope of the Main class, define the config values and the waypoints with the following syntax:<br/>
       
       WaypointSequence p = new WaypointSequence(10);
       p.new Waypoint(x_feet,y_feet,angle_radians); 
_the angles are absolute_<br/>
_If your angle was negative and the path didn't reticulate properly, try using a positive angle using invertY() on the Waypoint Sequence_
### Step 3:
Find the path file, and copy it to the location "://file/paths" on the rio
### Step 4:
Make sure that in your robot code that your path has been added to the array "kPathNames" and "KPathDescriptions" in AutoPaths.java
### Step 5:
Create an Autonomous Command/Command Group that uses the Follow Path command like this:<br/>
_new FollowPath(AutoPaths.get("TestPath"))_
### Step 6:
Run your auto!<br/><br/><br/>

This Uses FRC254's Spline Path Generating Code and FeedFoward Path Following. HUGE Thanks to them!!!:<br/>
https://github.com/Team254/TrajectoryLib




