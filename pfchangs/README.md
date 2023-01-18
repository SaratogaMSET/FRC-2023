# The "PF" stands for Particle Filter

## Todo list, in no particular order of priority

- [x] Get particle filter working with distance and orientation values from LL AprilTag locations - DONE
- [x] Display robot position in field w/ Shuffleboard/Glass - DONE
- [x] Get robot distance from landmarks using estimated pose from LL <-- for PF - DONE
- [ ] Tune forward, turn, and senseNoise values
- [X] Decouple translational motion from rotational motion in the particle filter for holonomic drivetrains
- [ ] Send raw/filtered positional data between the coprocessor and the RIO
- [ ] Resolve the issue of coprocessor threads not necessarily being in-sync with command scheduler loop (20 Hz vs way more)
- [ ] Related to the above (and possible solution): Update particle filter only on NT updates
- [ ] Convert Euler angles (from botpose) to rotation around a single, fixed axis in a 2D plane (around Z-axis only)
- [ ] Generate paths on-the-fly using PathPlanner to align with the nearest scoring location
- [ ] Refactor server to be a singleton + to start only once, even if start() is called multiple times
- [ ] Related to the above: start server on robot enable, rather than on robot startup
- [ ] Fix resampling to use raw distance data
- [ ] Move particle filter from util to pfchangs
- [ ] Automatically resample particle filter if (filtered - raw) > 2 and time elapsed from last resample > 2 <-- IGNORE THIS
- [ ] (?) Profile robot code to optimize particle filter (dubiously possible)

## Misc. Notes

### 3 main problems exist (besides latency compensation)

* Updating our localizer with raw distance values (ie. passing an array of distances--arbitrary length--to our resample method rather than doing what we're currently doing: sending bad pose data, then doing distance computations based off that bad pose data)
* Thread synchronization between client and server. (Essentially, we should treat the client code like it's inlined into the server thread's code, stopping the server thread from proceeding until after it receives filtered data from the localizer--treating 2 threads as one)
* What do we return when the robot is first initialized? (This may be a non-issue in Quixilver's implementation, but my belief is that when the match starts, there will be a delay between when the raw data is sent to the filter and when we get a measurement back from the filter. In the past, since everything was on the robot, this delay was so small as to be a non-issue--raw and filtered data were one and the same. Now, however, there is a chance we might want to use pose data before it's actually initialized, eg. for auto pathing. Say, at the start of the match, we start our auto path, but data from the localizer hasn't returned yet because of latency. What data do we return in that case? Raw pose data? Null? (This may be a non-issue with Quixilver's implementation, and if it still remains an issue, we should just leave resolving it to DT/Autos team rather than worry about it.))
