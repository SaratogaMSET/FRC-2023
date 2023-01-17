# The "PF" stands for Particle Filter

## Todo list, in no particular order of priority

- [x] Get particle filter working with distance and orientation values from LL AprilTag locations - DONE
- [x] Display robot position in field w/ Shuffleboard/Glass - DONE
- [x] Get robot distance from landmarks using estimated pose from LL <-- for PF - DONE
- [ ] Tune forward, turn, and senseNoise values
- [ ] Decouple translational motion from rotational motion in the particle filter for holonomic drivetrains
- [ ] Set up NetworkTables on this PC to communicate and send/receive values from the RIO and from LL
- [ ] Resolve the issue of coprocessor threads not necessarily being in-sync with command scheduler loop (20 Hz vs way more)
- [ ] Related to the above (and possible solution): Update particle filter only on NT updates
- [ ] Convert Euler angles (from botpose) to rotation around a single, fixed axis in a 2D plane (around Z-axis only)
- [ ] Generate paths on-the-fly using PathPlanner to align with the nearest scoring location
- [ ] Automatically resample particle filter if (filtered - raw) > 2 and time elapsed from last resample > 2 <-- IGNORE THIS
- [ ] (?) Profile robot code to optimize particle filter (dubiously possible)
