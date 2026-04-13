# 🚀 Current Development Focus

**Active Branch:** `refactor-test`
**Author:** shamsulhakimee
**Phase:** Lap 1 Challenge Sequencing

We are currently testing the Lap 1 challenge sequencing on the physical robot via the `refactor-test` branch. 

### What is being tested:
1. **Obstruction:** Clears obstacle, then triggers 1.5s re-centering delay before the roundabout.
2. **Roundabout:** Bot enters newly created `ROUNDABOUT` state and lane-follows the curve for 8.0s before exiting. 
3. **Tunnel:** LiDAR wall-follower takes over after roundabout.
4. **Sensor Gating:** Boom gate and traffic light are gated using `_boom_gate_armed` and `_tl_armed` flags to prevent them from firing prematurely.
5. **Encoder Fix:** Rear drive motor isolated for accurate odometry plotting.

> [!WARNING]
> Do not pull these changes into `main` until the track validation on `refactor-test` is confirmed successful!
