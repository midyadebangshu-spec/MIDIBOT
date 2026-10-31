# Third-party notices

MIDIBOT is proprietary (see `LICENSE`). The following third-party material is included
under its own license, which continues to apply to that material only.

| Material | Location in this repo | Source | License |
|---|---|---|---|
| Inspire RH56DFQ hand meshes, kinematics and inertials (decimated / converted) | `src/midibot_description/meshes/hands/`, `src/midibot_cad/data/inspire_hand_*.json` | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | BSD-3-Clause, Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"). Full text: `third_party/licenses/unitree_ros-BSD-3-Clause.txt` |
| Intel RealSense D435 mesh (decimated, converted to STL) | `src/midibot_description/meshes/sensors/d435.stl` | [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | Apache License 2.0. Full text: `third_party/licenses/realsense-ros-Apache-2.0.txt`. Modified: decimated and converted from COLLADA to STL. |

Vendor CAD (MyActuator, Livox, ROBOTIS) is **not** included; it's downloaded by each user
from the manufacturer with `src/midibot_cad/scripts/fetch_vendor_cad.py`, subject to the
manufacturer's terms.
