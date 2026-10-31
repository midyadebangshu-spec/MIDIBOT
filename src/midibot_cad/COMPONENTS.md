# MIDIBOT component CAD sources

How each electrical component is represented in the 3D model, where its CAD comes from, and whether it
can be committed to this repository.

| Component | Used for | CAD in the model | Source | In repo? |
|---|---|---|---|---|
| MyActuator RMD-X10-P20-200 | hip pitch, knee | Official STEP → mesh | myactuator.com X-series downloads | No: run `fetch_vendor_cad.py` (no redistribution license) |
| MyActuator RMD-X8-P20-120 | hip roll, waist yaw | Official STEP → mesh | myactuator.com | No: fetched |
| MyActuator RMD-X6-P20-60 | hip yaw, ankles (2×), shoulders, elbows | Official STEP → mesh | myactuator.com | No: fetched |
| MyActuator RMD-X4-P36-36 | shoulder yaw | Official STEP → mesh | myactuator.com | No: fetched |
| Livox Mid-360 | chest lidar | Official STEP → mesh | livoxtech.com (DJI CDN) | No: fetched |
| Inspire RH56DFQ hand | 5-finger hands | Meshes + kinematics + inertials | unitreerobotics/unitree_ros (BSD-3-Clause) | **Yes** (`meshes/hands`, `data/inspire_hand_*.json`) |
| Intel RealSense D435i | head RGB-D camera | Mesh | IntelRealSense/realsense-ros (Apache-2.0) | **Yes** (`meshes/sensors/d435.stl`) |
| ROBOTIS Dynamixel XH540-W270 | wrists | Placeholder box, datasheet size | ROBOTIS CAD is on www.robotis.com (blocked here) | Placeholder |
| ROBOTIS Dynamixel XM430-W350 | neck | Placeholder box, datasheet size | same as above | Placeholder |
| NVIDIA Jetson AGX Orin 64GB | AI Unit computer (chest) | Inertial only (inside shell) | developer.nvidia.com needs login | Placeholder |
| Intel NUC | Locomotion computer (pelvis) | Inertial only | - | Placeholder |
| VectorNav VN-100 | pelvis IMU | Inertial only | vectornav.com (download behind script) | Placeholder |
| 48 V battery, BMS, PDB, DC-DC, E-stop | power (torso) | Inertial only | - | Placeholder |
| ReSpeaker 4-mic array, speaker | audio (head/chest) | Inertial only | - | Placeholder |

Internal components that sit inside the shells only contribute mass and inertia (datasheet mass,
solid-box inertia at their mounting position). Externally visible components get a visual.

## Using the vendor meshes

```bash
pip install cadquery trimesh fast-simplification
python3 src/midibot_cad/scripts/fetch_vendor_cad.py      # writes midibot_description/meshes/vendor/
colcon build --symlink-install
ros2 launch midibot_gazebo sim.launch.py vendor_meshes:=true
```

Without the vendor meshes the actuators render as datasheet-sized cylinders; physics is identical
either way (mass/inertia never come from the vendor meshes).

## Regenerating the model

```bash
python3 src/midibot_cad/cad/structure.py            # structural parts -> STEP + STL
python3 src/midibot_cad/scripts/generate_urdf.py    # URDF/Xacro, ros2_control, controllers, mass budget
```

`import_open_meshes.py` re-imports the hand and camera meshes from shallow clones of the two open-source
repositories listed above.
