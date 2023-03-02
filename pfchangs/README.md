# The "PF" stands for Particle Filter

## Todo list, in no particular order of priority

- [x] Parse raw LL JSON dump/use LL library for multi-targeting
- [x] Replace/fix probability calc
- [ ] Fix heading probability calculation
- [ ] Account for gyro resets in motion update step
- [ ] Fix array indexing errors when adaptive particles is enabled
- [ ] Use headingErr and tx from LL to solve for particle position in the "circle of probabilities"
drawn around a single tag: compute headingErr first, then compute txErr (tx: use trig to solve for
particle tx, then substract for delta, absolute value)
- [ ] Send raw corner data from each tag over NT; use perspective projection to compute deltas between
expected and actual corner x/y coordinates for particles vs. LL
