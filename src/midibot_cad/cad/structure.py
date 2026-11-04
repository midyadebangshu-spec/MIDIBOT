#!/usr/bin/env python3
"""Parametric CadQuery models of MIDIBOT's structural parts.

Every part is modelled in its URDF link frame (millimetres here, metres on
export) so the exported meshes drop straight into the URDF with zero offset.
Off-the-shelf components (actuators, computers, sensors, hands) are NOT
modelled here; they come from vendor CAD or datasheet-sized placeholders.

Outputs:
  src/midibot_cad/step/<part>.step                     (for manufacturing)
  src/midibot_description/meshes/structure/<part>.stl  (metres, URDF visual)

Usage:  python3 structure.py
"""
import pathlib
import sys

import cadquery as cq

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import dims as D  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
STEP_DIR = HERE.parent / "step"
STL_DIR = HERE.parents[1] / "midibot_description" / "meshes" / "structure"

mm = 1000.0


def rbox(x, y, z, cx=0.0, cy=0.0, cz=0.0, fillet=0.0):
    """Rounded box, sizes and centre in metres."""
    b = cq.Workplane("XY").box(x * mm, y * mm, z * mm).translate((cx * mm, cy * mm, cz * mm))
    if fillet > 0:
        b = b.edges().fillet(fillet * mm)
    return b


def cyl(d, h, cx=0.0, cy=0.0, cz=0.0, axis="Z"):
    plane = {"Z": "XY", "Y": "XZ", "X": "YZ"}[axis]
    c = cq.Workplane(plane).circle(d / 2 * mm).extrude(h / 2 * mm, both=True)
    return c.translate((cx * mm, cy * mm, cz * mm))


def tapered(top_xy, bot_xy, z_top, z_bot, fillet=0.0):
    """Lofted rectangular tube (solid) from z_top down to z_bot."""
    s = (cq.Workplane("XY").workplane(offset=z_bot * mm).rect(bot_xy[0] * mm, bot_xy[1] * mm)
         .workplane(offset=(z_top - z_bot) * mm).rect(top_xy[0] * mm, top_xy[1] * mm).loft())
    if fillet > 0:
        s = s.edges().fillet(fillet * mm)
    return s


# ----------------------------------------------------------------- parts
def pelvis():
    body = rbox(0.150, 0.220, 0.090, cz=-0.005, fillet=0.02)
    # hip yaw motor sockets (shallow bosses under the pelvis)
    for s in (1, -1):
        body = body.union(cyl(0.090, 0.012, cy=s * D.HIP_Y, cz=-0.052))
    # waist flange on top
    return body.union(cyl(0.110, 0.012, cz=0.044))


def torso():
    shell = rbox(0.170, 0.280, 0.250, cz=0.150, fillet=0.03)
    waist = cyl(0.115, 0.070, cz=0.000)  # overlaps the pelvis flange (no visible gap)
    # shoulder bosses
    for s in (1, -1):
        shell = shell.union(cyl(0.085, 0.020, cy=s * 0.140, cz=D.SHOULDER_Z, axis="Y"))
    # neck collar
    collar = cyl(0.070, 0.035, cz=D.NECK_Z - 0.035 / 2)
    # chest recess for the lidar window
    recess = rbox(0.040, 0.100, 0.060, cx=0.085, cz=0.215)
    return shell.union(waist).union(collar).cut(recess)


def hip_yaw_link():
    return rbox(0.060, 0.055, 0.050, cz=-0.040, fillet=0.008)


def hip_roll_link():
    # yoke that carries the hip-pitch actuator (actuator axis along Y)
    return rbox(0.070, 0.040, 0.050, cz=0.0, fillet=0.008)


def thigh():
    t = tapered((0.090, 0.075), (0.072, 0.062), -0.050, -D.THIGH + 0.035, fillet=0.012)
    knee = cyl(0.070, 0.060, cz=-D.THIGH, axis="Y")
    # knee push-rod
    rod = rbox(0.012, 0.012, 0.170, cx=-0.040, cz=-0.215)
    return t.union(knee).union(rod)


def shank():
    s = tapered((0.070, 0.060), (0.052, 0.048), -0.030, -D.SHANK + 0.030, fillet=0.010)
    # twin ankle push-rods
    for y in (0.022, -0.022):
        s = s.union(rbox(0.010, 0.010, 0.200, cx=-0.045, cy=y, cz=-0.180))
    return s.union(cyl(0.050, 0.050, cz=-D.SHANK, axis="Y"))


def ankle_pitch_link():
    return rbox(0.032, 0.032, 0.032, fillet=0.004)


def foot():
    plate = rbox(D.FOOT_L, D.FOOT_W, D.FOOT_T, cx=D.FOOT_X, cz=-D.ANKLE_H + D.FOOT_T / 2, fillet=0.010)
    bracket = rbox(0.055, 0.060, 0.030, cz=-0.022, fillet=0.006)
    return plate.union(bracket)


def shoulder_pitch_link():
    return rbox(0.055, 0.050, 0.055, fillet=0.008)


def shoulder_roll_link():
    return rbox(0.062, 0.062, 0.090, cz=-0.050, fillet=0.010)


def shoulder_yaw_link():
    return tapered((0.060, 0.060), (0.055, 0.055), -0.020, D.ELBOW_Z + 0.025, fillet=0.010).union(
        cyl(0.055, 0.060, cz=D.ELBOW_Z, axis="Y"))


def elbow_link():
    return tapered((0.055, 0.055), (0.050, 0.050), -0.020, D.WRIST_YAW_Z + 0.010, fillet=0.008)


def wrist_yaw_link():
    return tapered((0.050, 0.050), (0.042, 0.042), -0.010, D.WRIST_PITCH_Z + 0.020, fillet=0.008)


def wrist_pitch_link():
    return rbox(0.026, 0.026, 0.026, fillet=0.004)


def wrist_roll_link():
    return cyl(0.050, 0.012, cz=D.HAND_MOUNT_Z + 0.006).union(rbox(0.030, 0.040, 0.025, cz=-0.012, fillet=0.004))


def neck_link():
    return cyl(0.040, 0.030, cz=0.015)


def head():
    skull = rbox(0.140, 0.130, 0.120, cx=0.005, cz=0.040, fillet=0.035)
    visor = rbox(0.030, 0.105, 0.040, cx=0.075, cz=0.055, fillet=0.010)
    return skull.cut(visor)


PARTS = {
    "pelvis": pelvis, "torso": torso, "hip_yaw_link": hip_yaw_link, "hip_roll_link": hip_roll_link,
    "thigh": thigh, "shank": shank, "ankle_pitch_link": ankle_pitch_link, "foot": foot,
    "shoulder_pitch_link": shoulder_pitch_link, "shoulder_roll_link": shoulder_roll_link,
    "shoulder_yaw_link": shoulder_yaw_link, "elbow_link": elbow_link, "wrist_yaw_link": wrist_yaw_link,
    "wrist_pitch_link": wrist_pitch_link, "wrist_roll_link": wrist_roll_link, "neck_link": neck_link,
    "head": head,
}


def main():
    import trimesh

    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    for name, fn in PARTS.items():
        part = fn()
        cq.exporters.export(part, str(STEP_DIR / f"{name}.step"))
        tmp = STL_DIR / f"{name}.stl"
        cq.exporters.export(part, str(tmp), tolerance=0.2, angularTolerance=0.2)
        m = trimesh.load(str(tmp), force="mesh")
        m.apply_scale(0.001)
        m.export(str(tmp))
        print(f"{name:22s} {len(m.faces):6d} faces  volume {m.volume * 1e6:8.1f} cm^3")


if __name__ == "__main__":
    main()
