# MIDIBOT — Humanoid Robot Plan (v1.1)

_v1.1 (Phase 1 done): actuators fixed to MyActuator RMD-X V4 models from the official datasheets; mass target revised to ~39 kg; see [MASS_BUDGET.md](MASS_BUDGET.md)._

## 1. Locked decisions

| Decision | Choice |
|---|---|
| Height / mass | 1.31 m / 39.4 kg (model; was 35 kg before real actuator masses were known) |
| Actuators | Research-grade quasi-direct-drive (QDD) BLDC with integrated drivers |
| Hands | 5-finger dexterous hands |
| Walking | LIPM + ZMP preview control + whole-body IK (CPU-friendly) |
| Middleware / sim | ROS 2 Jazzy, Gazebo Harmonic, ros2_control (Ubuntu 24.04) |
| Architecture | Two units: **Locomotion & Movement Unit** and **AI Unit** |

## 2. Kinematics & degrees of freedom

| Group | Joints | DOF |
|---|---|---|
| Legs (×2) | hip yaw, hip roll, hip pitch, knee, ankle pitch, ankle roll | 12 |
| Waist | yaw | 1 |
| Arms (×2) | shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll | 14 |
| Head | pan, tilt | 2 |
| Hands (×2) | Inspire RH56DFQ: 6 actuated each (thumb rotation, thumb bend, index, middle, ring, little) + 6 linkage-coupled joints each | 12 actuated (24 joints) |
| **Total** | | **41 actuated** |

Segment lengths (source of truth: `src/midibot_cad/cad/dims.py`):
thigh 0.30 m, shank 0.30 m, ankle height 0.06 m, foot 0.22 × 0.11 m, hip width 0.18 m,
pelvis→shoulder 0.355 m, upper arm 0.22 m, forearm 0.20 m, shoulder width 0.35 m, pelvis height 0.77 m.

## 3. Unit 1 — Locomotion & Movement Unit

Owns the body, real-time control and **final safety authority**.

### 3.1 Actuators (selected; values from manufacturer datasheets)

| Joints | Actuator | Rated / peak torque | Mass |
|---|---|---|---|
| Hip pitch, knee (via push-rod) | MyActuator RMD-X10-P20-200 | 60 / 170 Nm | 1.64 kg |
| Hip roll, waist yaw | MyActuator RMD-X8-P20-120 | 43 / 120 Nm | 1.40 kg |
| Hip yaw, ankles (2 per leg), shoulder pitch/roll, elbow | MyActuator RMD-X6-P20-60 | 20 / 60 Nm | 0.82 kg |
| Shoulder yaw | MyActuator RMD-X4-P36-36 | 10.5 / 34 Nm | 0.36 kg |
| Wrist yaw/pitch/roll | ROBOTIS Dynamixel XH540-W270 | ~10.6 Nm stall | 0.165 kg |
| Neck yaw, head pitch | ROBOTIS Dynamixel XM430-W350 | ~4.1 Nm stall | 0.082 kg |
| Hands | Inspire RH56DFQ (6 linear micro-actuators) | ~10 N fingertip | ~0.54 kg per hand |

Ankle pitch/roll will use a parallel two-motor push-rod linkage (motors mounted on the shank)
to reduce distal mass — in simulation this is modelled as serial joints with equivalent limits.

### 3.2 Electronics

| Subsystem | Component |
|---|---|
| Locomotion computer | Intel NUC (i7) with PREEMPT_RT kernel |
| Joint bus | 4× CAN-FD (left leg, right leg, left arm+waist, right arm); USB-CAN-FD adapters or STM32H7 bridge |
| Hands / head bus | RS485 (hands), TTL/RS485 Dynamixel bus (head) |
| Body IMU | VectorNav VN-100 (pelvis) |
| Foot sensing | 4-point load cells per foot (or 6-axis F/T sensor) |
| Power | 48 V Li-ion pack (~10 Ah) + BMS, power distribution board, DC-DC 48→24/12/5 V |
| Safety | Hardware E-stop (wireless + on-body) cutting motor power via contactor |

### 3.3 Software
- `ros2_control` hardware interface (sim: `gz_ros2_control`; real: CAN-FD driver)
- Joint-level PD / torque control at 1 kHz
- **Walking:** footstep planner → LIPM ZMP preview controller (CoM trajectory) → swing-foot trajectories → whole-body IK → joint targets; ankle/hip strategy balance feedback
- State estimator: IMU + leg kinematics (+ contact detection from load cells)
- Safety supervisor: joint/torque/temperature limits, fall detection, safe sit-down, E-stop handling

## 4. Unit 2 — AI Unit

Perception, intelligence, interaction. Sends **goals only**; never drives joints directly.

| Subsystem | Component |
|---|---|
| Compute | NVIDIA Jetson AGX Orin 64 GB (chest) |
| Head vision | Intel RealSense D435i (stereo depth + RGB + IMU) |
| 3D lidar | Livox Mid-360 (chest window, front) |
| Wrist cameras (optional) | small global-shutter USB cameras |
| Audio | ReSpeaker USB 4-mic array + speaker |
| Network | Gigabit Ethernet to Unit 1, Wi-Fi 6 to operator |

Software: object detection/segmentation, human tracking, SLAM (RTAB-Map / slam_toolbox),
footstep-level navigation, MoveIt 2 arm + hand grasp planning, speech (STT/TTS),
LLM task planner → BehaviorTree.CPP executor.

## 5. Inter-unit interface

```
 AI UNIT (Jetson, ROS namespace /ai)
    │  Ethernet · ROS 2 DDS
    │  ↓ /loco/cmd_vel, /loco/footstep_plan, /loco/arm_goal (action),
    │    /loco/hand_cmd, /loco/head_cmd, /loco/mode (stand/walk/sit)
    │  ↑ /loco/odom, /joint_states, /imu, /foot_contacts, /loco/status
 LOCOMOTION UNIT (NUC, ROS namespace /loco)
    │  CAN-FD 1 kHz · RS485
 Actuators · IMU · foot sensors · hands
```

Custom interfaces live in `midibot_interfaces`. The locomotion unit validates and may reject
every AI command. In simulation both units run as separate namespaces/processes.

## 6. CAD strategy

- **Electrical/off-the-shelf components:** download manufacturer STEP files (Jetson AGX Orin,
  RealSense D435i, Livox Mid-360, CubeMars/MyActuator motors, Dynamixel, Inspire hand, VectorNav),
  convert to meshes (visual: DAE/STL, decimated; collision: primitives/convex hulls).
  Where a file is unavailable or its license forbids redistribution, use a dimensionally accurate
  placeholder and record it in `midibot_cad/COMPONENTS.md`.
- **Structure (frame, brackets, shells, feet):** parametric CadQuery scripts, exportable to STEP
  for manufacturing.
- **Mass properties:** datasheet masses for components, CAD-derived inertia for structure;
  totals tracked in a mass budget.

### Mass budget

Generated from the model: **39.4 kg** (actuators 22.5, structure 10.0, power 3.7, electronics 1.7,
hands 1.1, sensors 0.4). Full breakdown: [MASS_BUDGET.md](MASS_BUDGET.md).

## 7. Repository layout

```
MIDIBOT/
├── docs/                     # this plan, design notes
└── src/
    ├── midibot_description/  # URDF/Xacro, meshes
    ├── midibot_cad/          # CadQuery sources, STEP→mesh pipeline, COMPONENTS.md
    ├── midibot_gazebo/       # worlds, sensor plugins, ros_gz bridge
    ├── midibot_control/      # ros2_control configs, controllers
    ├── midibot_locomotion/   # footstep planner, ZMP walking, state estimation, balance
    ├── midibot_safety/       # supervisor, limits, E-stop
    ├── midibot_hands/        # hand controllers, grasp primitives
    ├── midibot_ai/           # perception, behavior trees, LLM planner, speech
    ├── midibot_manipulation/ # MoveIt 2 config
    ├── midibot_interfaces/   # msgs / srvs / actions
    └── midibot_bringup/      # launch: sim, loco-only, ai-only, full
```

## 8. Phases

| Phase | Deliverable | Done when |
|---|---|---|
| 1. Model ✅ | CAD, meshes, URDF/Xacro with correct mass/inertia | Robot loads in Gazebo, holds pose with joints locked, foot load = model weight |
| 2. Control | ros2_control for all 41 actuators, standing balance | Stands unassisted, recovers from small pushes |
| 3. Walking | ZMP walking driven by `/loco/cmd_vel` | Walks forward/backward, turns, stops on flat ground |
| 4. AI Unit | Sensors, SLAM, navigation, MoveIt reach + 5-finger grasp | Navigates to a goal; picks up an object from a table |
| 5. Integration | Command → LLM planner → behavior tree → actions | "Bring me the cup" works end-to-end in sim |

## 9. Simulation notes
- Physics engine: DART (Gazebo default). DART can't enforce URDF `<mimic>` constraints, so the
  finger linkages are reproduced by `midibot_control/hand_coupler` (6 commands → 12 joints per hand).
  Bullet-Featherstone supports mimic but was tested and rejected: the robot fell over and the foot F/T sensors didn't work.
- Rendering sensors (camera, lidar) run on CPU here; with them on, the sim runs at ~0.3× real time.

## 10. Known risks
- ZMP walking is sensitive to mass/inertia accuracy — Phase 1 mass properties matter.
- 41 actuators + dexterous hands make Gazebo slower than real time on CPU; hands may be simplified
  (fewer collision shapes) during walking tests.
- Some vendor CAD files may be unavailable or non-redistributable; placeholders will be used.
