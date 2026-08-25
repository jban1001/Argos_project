# ARGOS MPPI controller profile

`config/nav2_params.yaml` keeps the hardware-tested RPP profile. The MPPI
profile is isolated in `config/nav2_params_mppi.yaml` and is started with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ydlidar_ws/install/setup.bash
source ~/argos_project/ros2_ws/install/setup.bash
ros2 launch argos_bringup argos_navigation_mppi.launch.py
```

Return to RPP at any time with:

```bash
ros2 launch argos_bringup argos_navigation.launch.py
```

## Safety-first hardware validation

1. Put the tracks off the floor and verify `/cmd_vel`, wheel direction, and the
   0.5 s base-driver watchdog before any floor test.
2. On the floor, start in an open area with a short goal (0.5-1.0 m) and an
   operator ready to stop the base driver.
3. Confirm `controller_server` sustains 8 Hz without missed-rate warnings.
4. Place a stationary obstacle well outside the footprint, then move it into
   the local costmap and confirm deceleration or local avoidance.
5. Only after these pass, tune one group at a time: sampling stddevs, CostCritic
   versus PathFollowCritic, then PathAlignCritic.

The initial profile is forward-only. Reverse expansion remains disabled in the
State Lattice planner, preventing MPPI from introducing an unvalidated reverse
maneuver. The local and global costmaps retain the measured rectangular
footprint and LiDAR obstacle layer.
