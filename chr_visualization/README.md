# CHR ground-station visualization

This package runs only on the ground-station NUC. It is read-only: it does not
publish flight commands, Dynamixel commands, torque requests, control flags, or
keyboard commands.

## Coordinate convention

The entire RViz model uses the controller's kinematic convention:

- `world_enu`: ENU world, +Z up
- `palletrone`: FLU body, +Z up
- `tcp`: FK frame produced by the arm chain

`/data_logging_msg` is generated from PX4 NED/FRD data. `chr_rviz` converts its
position, attitude, velocities, acceleration, torque, and CoM data to ENU/FLU
before publishing visualization TF and markers. `/chr/state/tcp_pose` and
`/chr/target/tcp_pose` already use `world_enu` and are not converted again.

## Model source

The model is built from the validated simulator `chr_description` package:

- Palletrone `BODY.stl` and four `PROP.stl` instances
- the exact X-configuration rotor positions and rotations from `palletrone.xml`
- the arm transforms and meshes from `arm.xml`
- arm mount `(-0.10, 0, -0.13) m` in the Z-up Palletrone frame
- J1 and J3 limits ±90 degrees; J2 limit 0–30 degrees

The original `px4_visual` content is retained: current/desired base frames,
linear velocity, acceleration, angular velocity, desired torque, default CoM,
and updated CoM. Current and target TCP paths are added. In the TF display,
Palletrone and TCP are enabled; arm and rotor link frames are disabled.

## Build and run

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/chr_gsc_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch chr_visualization chr_visualization.launch.py
```

Keyboard teleoperation and rosbag recording remain separate onboard processes.
They are deliberately absent from this launch file.

When the hardware is disconnected, the default `preview_without_data:=true`
shows the validated model at 1.2 m with all joints at zero. It prints warnings
that the pose and joint values are previews. To show only received experiment
data, run:

```bash
ros2 launch chr_visualization chr_visualization.launch.py preview_without_data:=false
```

For recorded bags:

```bash
ros2 launch chr_visualization chr_visualization.launch.py \
  use_sim_time:=true preview_without_data:=false
ros2 bag play /path/to/rosbag2_YYYY_MM_DD-HH_MM_SS --clock
```

## Expected onboard topics

- `/data_logging_msg`
- `/cone/joint_states`
- `/chr/state/tcp_pose`
- `/chr/target/tcp_pose`

The visualization node publishes only `/tf` and topics under
`/chr/visualization/`.
