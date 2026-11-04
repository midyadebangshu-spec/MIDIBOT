# MIDIBOT: instructions for Claude

MIDIBOT is a proprietary (closed-source) 1.31 m, 39 kg humanoid robot with 41 actuated joints,
simulated in ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04. Design plan and roadmap: `docs/PLAN.md`.

## Working rules

1. **Plan before any multi-step task.** Before starting work that takes more than one step, write
   a short plan: the goal, the steps, the files involved, and how the result will be verified. Track
   the steps with the task list and keep it updated as work progresses.
2. **Commit the work.** Commit each finished, verified piece of work with a clear message, and push
   it. Don't leave completed work uncommitted. Never commit secrets (API keys, tokens) or vendor CAD
   files (see below).
3. **One branch per phase.** Each roadmap phase gets its own branch (`phase1-robot-model`,
   `phase2-control`, `phase3-walking`, ...), created from the latest `main` and merged by pull request.
   Don't commit directly to `main`.
4. **Verify in simulation before calling something done.** Run the relevant acceptance test and
   report its real output, failures included.

## Repository layout

```
src/
├── midibot_cad/          # CadQuery structure, vendor CAD fetcher, URDF generator (not a ROS package)
├── midibot_description/  # URDF/Xacro, meshes, ros2_control + controller config (mostly GENERATED)
├── midibot_gazebo/       # world, sim launch, bridge, check/push/snapshot/video tools
└── midibot_control/      # balance controller, hand coupler
```

## Key conventions

- **Generated files: don't edit by hand.** `midibot.urdf.xacro`, `midibot.ros2_control.xacro`,
  `ros2_controllers.yaml`, `hand_coupling.json` and `docs/MASS_BUDGET.md` come from
  `src/midibot_cad/scripts/generate_urdf.py`. Change the generator (or `cad/dims.py`,
  `cad/structure.py`), then rerun it. The CAD scripts run with the CadQuery Python
  (`/usr/local/bin/python3` in the cloud container), not ROS's Python 3.12.
- **Vendor CAD stays out of git.** `meshes/vendor/*.stl` is fetched by `fetch_vendor_cad.py`, because the
  vendor licenses don't allow redistribution. Third-party meshes that are committed must be listed in
  `THIRD_PARTY_NOTICES.md`.
- **Two units:** the Locomotion & Movement Unit (`/loco`) has final safety authority, and the AI Unit (`/ai`)
  only sends goals.
- **Python control nodes:** avoid high-rate, large ROS messages and Gazebo's 1 kHz `/clock`. Take timing
  from message stamps. Posture transitions must start from the controller's current setpoints, not from
  the measured joint angles. See the lessons learned in `docs/PLAN.md`.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
ros2 launch midibot_gazebo sim.launch.py          # headless; balance controller on
ros2 run midibot_gazebo push_test.py              # Phase 2 acceptance (balance + pushes)
ros2 launch midibot_gazebo sim.launch.py balance:=false   # then: ros2 run midibot_gazebo check_model.py
```
