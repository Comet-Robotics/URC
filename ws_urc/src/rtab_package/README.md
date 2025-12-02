# `rtab_package`

`rtab_package` fuses Odometry and SLAM information which integrates:

- Robot localization using an UKF with IMU and SLAM data
- A preconfigured RViz visualization
- RGB-D SLAM visual odometry

---

### Configuration

- `ukf.yaml`: Produces real-time filtered localization by fusing odometry and SLAM

- `config.rviz`: RViz Setup for map, robot, and odometry
- `rtabmap_config.yaml` (unused): SLAM configuration
- `navsat_transform.yaml` (unused): GPS configuration

### Launching the SLAM stack

```bash
ros2 launch rtab_package bringup.launch.py
```

This will launch:

- rgbd_odometry
- ukf_node
- rtabmap

### Preconfigured RViz Visualization

Includes a `config.rviz` for instant visualization of:

- Robot model
- Occupancy map
- Odometry tracks
- Navigation goal tools
