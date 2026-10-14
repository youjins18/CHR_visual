# CHR Ground-Station Workspace

`chr_gsc_ws` is the ground-station workspace for the **Cone Harvest Robot
(CHR)**.

This workspace provides the validated robot model and an RViz monitor for live
experiments or recorded bags. It is visualization-only: it does not send flight,
Dynamixel, torque, controller-toggle, or keyboard commands to the onboard
computer.

## Packages

```text
chr_gsc_ws/src/
├── chr_description/     # CHR geometry, meshes, URDF and MuJoCo model
└── chr_visualization/   # ROS 2 transforms, markers, RViz configuration and launch
```

### `chr_description`

The description package is derived from the validated simulator model.

- `mujoco/scene.xml`: visualization defaults and complete scene entry point
- `mujoco/chr.xml`: physics options, assets, actuators and model composition
- `mujoco/palletrone.xml`: free base, four diagonal tilt joints, rotors and CAD
- `mujoco/arm.xml`: arm inertial data, CAD meshes and joints J1--J3
- `urdf/chr.urdf`: robot model used by RViz

### `chr_visualization`

The visualization package displays:

- the complete Palletrone and arm model
- current and desired Palletrone frames
- linear velocity and acceleration
- angular velocity and desired torque
- default and updated centre of mass
- current and target TCP paths
- Palletrone and TCP frames by default

Individual arm-joint and rotor-link frames are available in RViz but disabled by
default. The desired TCP is shown as a blue position sphere and an arrow along
the desired TCP frame's local +Z direction.

## Coordinate frames

The RViz model follows the controller's kinematic convention:

- `world_enu`: ENU world frame, +Z up
- `palletrone`: FLU body frame, +Z up
- `tcp`: end-effector frame calculated by the arm forward kinematics

## Expected onboard topics

The ground station subscribes to:

- `/data_logging_msg`
- `/cone/joint_states`
- `/chr/state/tcp_pose`
- `/chr/target/tcp_pose`

The visualization node publishes only `/tf` and topics under
`/chr/visualization/`. Keyboard teleoperation and rosbag recording remain
separate onboard processes and are intentionally absent from the visualization
launch file.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/chr_gsc_ws
colcon build --symlink-install
source install/setup.bash
```

## Live visualization

```bash
ros2 launch chr_visualization chr_visualization.launch.py
```

When no hardware data is received, the default
`preview_without_data:=true` setting displays the model at 1.2 m with all arm
joints at zero. Console warnings identify the pose and joint values as previews;
this is not a measured robot state.

To keep the model hidden until real experiment data arrives, run:

```bash
ros2 launch chr_visualization chr_visualization.launch.py \
  preview_without_data:=false
```

## TREE 
```bash
├── chr_description
│   ├── CMakeLists.txt
│   ├── meshes
│   │   ├── arm
│   │   │   ├── base_link.STL
│   │   │   ├── link1.STL
│   │   │   ├── link2.STL
│   │   │   ├── link3.STL
│   │   │   └── link4.STL
│   │   └── palletrone
│   │       ├── BODY.stl
│   │       └── PROP.stl
│   ├── mujoco
│   │   ├── arm.xml
│   │   ├── chr.xml
│   │   ├── palletrone.xml
│   │   └── scene.xml
│   ├── package.xml
│   └── urdf
│       └── chr.urdf
├── chr_visualization
│   ├── CMakeLists.txt
│   ├── config
│   │   └── chr_visualization.rviz
│   ├── launch
│   │   └── chr_visualization.launch.py
│   ├── package.xml
│   └── src
│       └── chr_rviz.cpp
└── README.md
```

