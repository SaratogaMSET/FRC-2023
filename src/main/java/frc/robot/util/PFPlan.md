# Particle Filter Plan

## In no particular order of priority

* Get particle filter working with distance and orientation values from LL AprilTag locations - DONE
* Display robot position in field w/ Shuffleboard/Glass - DONE
* Get robot distance from landmarks using estimated pose from LL <-- for PF - DONE
* Tune forward, turn, and senseNoise values
* Decouple translational motion from rotational motion in the particle filter
* Set up NetworkTables on this PC to communicate and send/receive values from the RIO and from LL
* Resolve the issue of coprocessor threads not neessarily being in-sync with command scheduler loop (20 Hz vs way more)
* Related to the above (and possible solution): figure out how to update particle filter only on NT updates
* (?) Profile robot code to optimize particle filter (dubiously possible)
