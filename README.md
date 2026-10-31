# MIDIBOT

A 1.31 m, 39 kg humanoid robot with 41 actuated joints, simulated in ROS 2 Jazzy and Gazebo Harmonic.

![MIDIBOT in Gazebo](docs/images/midibot_gazebo.png)

MIDIBOT is split into two units:

1. **Locomotion & Movement Unit**: legs, arms, waist, head and 5-finger hands; real-time control, ZMP walking
   and the safety supervisor. Runs on the Intel NUC in the pelvis (ROS namespace `/loco`).
2. **AI Unit**: perception, SLAM, manipulation planning, speech and an LLM task planner. Runs on the
   Jetson AGX Orin in the chest (ROS namespace `/ai`).

See [docs/PLAN.md](docs/PLAN.md) for the design plan and roadmap, and [docs/MASS_BUDGET.md](docs/MASS_BUDGET.md)
for the mass breakdown and actuator assignment.

## Status

| Phase | State |
|---|---|
| 1. Model: CAD, meshes, URDF, mass properties | ✅ done |
| 2. Control: ros2_control effort control, standing balance | next |
| 3. ZMP walking | planned |
| 4. AI Unit: sensors, SLAM, navigation, grasping | planned |
| 5. Integration: command → LLM planner → behaviour tree | planned |

## Repository layout

```
src/
├── midibot_cad/          # CadQuery structure, vendor CAD fetcher, URDF generator (not a ROS package)
├── midibot_description/  # URDF/Xacro, meshes, ros2_control + controller config
├── midibot_gazebo/       # Gazebo world, sim launch, bridge config, check + snapshot tools
└── midibot_control/      # control nodes (hand coupler; walking and balance come later)
```

## Quick start (Ubuntu 24.04, ROS 2 Jazzy)

Install the dependencies:

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-rviz2 python3-colcon-common-extensions
```

Build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the simulation (`headless:=false` opens the Gazebo GUI):

```bash
ros2 launch midibot_gazebo sim.launch.py headless:=false
```

In a second terminal, run the Phase 1 acceptance check:

```bash
ros2 run midibot_gazebo check_model.py
```

Optional: download the official vendor CAD (motors, lidar) for realistic visuals, then launch with
`vendor_meshes:=true`. See [src/midibot_cad/COMPONENTS.md](src/midibot_cad/COMPONENTS.md).

```bash
python3 src/midibot_cad/scripts/fetch_vendor_cad.py
```

## Interfaces (Phase 1)

| Topic / action | Type | Unit |
|---|---|---|
| `/body_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 29 body joints |
| `/loco/{left,right}_hand/command` | `std_msgs/Float64MultiArray` (6 values: thumb yaw, thumb pitch, index, middle, ring, pinky) | hands |
| `/joint_states` | `sensor_msgs/JointState` | all 53 joints |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | pelvis IMU |
| `/loco/{left,right}_foot_ft` | `geometry_msgs/WrenchStamped` | ankle force/torque |
| `/ai/camera/{image,depth,camera_info}` | `sensor_msgs/Image`, `CameraInfo` | head RGB-D |
| `/ai/lidar/points` | `sensor_msgs/PointCloud2` | chest lidar |

## License

**Proprietary. All rights reserved.** See [LICENSE](LICENSE). No use, copying or distribution without
the copyright holder's written permission.

Third-party material (Inspire hand meshes, BSD-3-Clause; RealSense mesh, Apache-2.0) stays under its own
license; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Vendor STEP files are downloaded by each user
and never redistributed.
