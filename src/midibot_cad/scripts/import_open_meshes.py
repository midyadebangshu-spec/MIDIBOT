#!/usr/bin/env python3
"""Import permissively licensed component meshes into midibot_description.

Sources (cloned with git, shallow):
  * Inspire RH56DFQ dexterous hand meshes - unitreerobotics/unitree_ros
    (BSD-3-Clause), robots/g1_description/meshes/{L,R}_hand_base_link.STL, Link11..22_{L,R}.STL
  * Intel RealSense D435(i) mesh - IntelRealSense/realsense-ros (Apache-2.0),
    realsense2_description/meshes/d435.dae

Meshes are decimated so the repository stays small. Usage:
  python3 import_open_meshes.py --unitree /path/to/unitree_ros --realsense /path/to/realsense-ros
"""
import argparse
import pathlib

import trimesh

HERE = pathlib.Path(__file__).resolve().parent
DESC = HERE.parents[1] / "midibot_description" / "meshes"


def decimate(mesh, faces):
    if len(mesh.faces) > faces:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=faces)
        except Exception as exc:  # fall back to the original mesh
            print("  decimation failed:", exc)
    return mesh


def _floats(s):
    return [float(v) for v in s.split()]


def extract_hand_kinematics(unitree: pathlib.Path):
    """Save the DFQ hand links/joints as JSON so the URDF generator is self-contained."""
    import json
    import xml.etree.ElementTree as ET

    data_dir = HERE.parent / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    for side, fname in (("L", "DFQ_left_hand.urdf"), ("R", "DFQ_right_hand.urdf")):
        root = ET.parse(unitree / "robots" / "g1_description" / "inspire_hand" / fname).getroot()
        links, joints = {}, []
        for ln in root.findall("link"):
            inr = ln.find("inertial")
            vis = ln.find("visual/geometry/mesh")
            if inr is None or vis is None:
                continue
            o = inr.find("origin")
            i = inr.find("inertia").attrib
            links[ln.get("name")] = {
                "mesh": pathlib.Path(vis.get("filename")).stem,
                "mass": float(inr.find("mass").get("value")),
                "com": _floats(o.get("xyz", "0 0 0")),
                "inertia": [float(i[k]) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")],
            }
        for j in root.findall("joint"):
            if j.get("type") == "fixed":
                continue
            o = j.find("origin")
            lim = j.find("limit").attrib
            mim = j.find("mimic")
            joints.append({
                "name": j.get("name"), "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": _floats(o.get("xyz", "0 0 0")), "rpy": _floats(o.get("rpy", "0 0 0")),
                "axis": _floats(j.find("axis").get("xyz")),
                "lower": float(lim["lower"]), "upper": float(lim["upper"]),
                "mimic": None if mim is None else {
                    "joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1)),
                    "offset": float(mim.get("offset", 0))},
            })
        out = data_dir / f"inspire_hand_{side}.json"
        out.write_text(json.dumps({"source": "unitreerobotics/unitree_ros (BSD-3-Clause) " + fname,
                                   "links": links, "joints": joints}, indent=1))
        print(f"hand {side}: {len(links)} links, {len(joints)} joints, "
              f"mass {sum(v['mass'] for v in links.values()):.3f} kg -> {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unitree", type=pathlib.Path, required=True)
    ap.add_argument("--realsense", type=pathlib.Path, required=True)
    a = ap.parse_args()

    src = a.unitree / "robots" / "g1_description" / "meshes"
    out = DESC / "hands"
    out.mkdir(parents=True, exist_ok=True)
    names = [f"{s}_hand_base_link" for s in "LR"] + [f"Link{i}_{s}" for i in range(11, 23) for s in "LR"]
    for n in names:
        m = trimesh.load(str(src / f"{n}.STL"), force="mesh")
        m = decimate(m, 3000 if "base" in n else 1200)
        m.export(str(out / f"{n}.stl"))
        print(f"{n}: {len(m.faces)} faces")

    extract_hand_kinematics(a.unitree)

    rs = a.realsense / "realsense2_description" / "meshes" / "d435.dae"
    m = trimesh.load(str(rs), force="mesh")
    m = decimate(m, 6000)
    # the realsense mesh is authored in metres with X forward
    (DESC / "sensors").mkdir(parents=True, exist_ok=True)
    m.export(str(DESC / "sensors" / "d435.stl"))
    print(f"d435: {len(m.faces)} faces, extents {m.extents}")


if __name__ == "__main__":
    main()
