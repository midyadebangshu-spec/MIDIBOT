# MIDIBOT — Humanoid Robot Plan (v1.0)

## 1. Locked decisions

| Decision | Choice |
|---|---|
| Height / mass | 1.30 m / ~35 kg target |
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
| Hands (×2) | 6 actuated each (thumb rotation, thumb bend, index, middle, ring, little); 12 joints each via mimic/linkage coupling | 12 actuated (24 joints) |
| **Total** | | **41 actuated** |

Approximate segment lengths (scaled from human proportions at 1.30 m, to be refined in CAD):
thigh 0.30 m, shank 0.30 m, ankle height 0.06 m, foot 0.20 × 0.10 m, hip width 0.18 m,
torso (pelvis→shoulder) 0.42 m, upper arm 0.22 m, forearm 0.20 m, shoulder width 0.34 m.

## 3. Unit 1 — Locomotion & Movement Unit

Owns the body, real-time control and **final safety authority**.

### 3.1 Actuators (candidates — torque figures approximate, verify against datasheets)

| Joints | Candidate | Approx. peak torque | Approx. mass |
|---|---|---|---|
| Knee, hip pitch | MyActuator RMD-X10 / CubeMars AK80-64 class | 100–120 Nm | 0.9–1.2 kg |
| Hip roll, hip yaw, ankle pitch/roll | CubeMars AK80-9 / AK70-10 class | 18–25 Nm (ankles via linkage for more) | 0.5–0.6 kg |
| Waist yaw, shoulders, elbow | CubeMars AK60-6 / MyActuator RMD-X6 class | 9–20 Nm | 0.35–0.6 kg |
| Wrist (3) | MyActuator RMD-X4 / Dynamixel XH540 class | 3–10 Nm | 0.2–0.35 kg |
| Head pan/tilt | Dynamixel XM430-W350 | ~4 Nm | 0.08 kg |
| Hands | Inspire RH56-class dexterous hand (6 linear micro-actuators, RS485) | ~10 N fingertip | ~0.55 kg per hand |

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
| 3D lidar | Livox Mid-360 (upper chest) |
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

### Mass budget (target)

| Item | kg |
|---|---|
| Actuators (29 body) | 16.5 |
| Hands (2) | 1.1 |
| Structure (aluminium/CF) | 8.0 |
| Battery + BMS | 3.5 |
| Computers + sensors | 2.5 |
| Shells, wiring, misc. | 3.4 |
| **Total** | **35.0** |

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
| 1. Model | CAD, meshes, URDF/Xacro with correct mass/inertia | Robot loads in Gazebo, holds pose with joints locked, mass ≈ 35 kg |
| 2. Control | ros2_control for all 41 actuators, standing balance | Stands unassisted, recovers from small pushes |
| 3. Walking | ZMP walking driven by `/loco/cmd_vel` | Walks forward/backward, turns, stops on flat ground |
| 4. AI Unit | Sensors, SLAM, navigation, MoveIt reach + 5-finger grasp | Navigates to a goal; picks up an object from a table |
| 5. Integration | Command → LLM planner → behavior tree → actions | "Bring me the cup" works end-to-end in sim |

## 9. Known risks
- ZMP walking is sensitive to mass/inertia accuracy — Phase 1 mass properties matter.
- 41 actuators + dexterous hands make Gazebo slower than real time on CPU; hands may be simplified
  (fewer collision shapes) during walking tests.
- Some vendor CAD files may be unavailable or non-redistributable; placeholders will be used.
