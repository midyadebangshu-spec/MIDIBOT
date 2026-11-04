#!/usr/bin/env python3
"""Generate MIDIBOT's URDF/Xacro, controller config and mass budget.

Each link's inertial is the exact composite of:
  * its structural part (CadQuery mesh, inertia from geometry scaled to a target mass),
  * the actuators whose housings are mounted on it (datasheet mass, solid-cylinder inertia),
  * internal electronics (datasheet mass, solid-box inertia),
  * for the hands, the Inspire RH56DFQ link inertials plus the palm actuator mass.

Outputs (all committed; re-run after changing dims.py / structure.py / this file):
  src/midibot_description/urdf/midibot.urdf.xacro
  src/midibot_description/config/ros2_controllers.yaml
  docs/MASS_BUDGET.md

Usage:  python3 generate_urdf.py
"""
import json
import math
import pathlib
import sys

import numpy as np
import trimesh

HERE = pathlib.Path(__file__).resolve().parent
CAD = HERE.parent
REPO = CAD.parents[1]
DESC = REPO / "src" / "midibot_description"
sys.path.insert(0, str(CAD / "cad"))
import dims as D  # noqa: E402

PKG = "package://midibot_description"

# ------------------------------------------------------------------ catalogue
# Actuators: datasheet values (MyActuator V4 parameter sheets, ROBOTIS e-Manual).
# dims: cylinder (diameter, length) or box (x, y, z) with z = output axis.
ACT = {
    "X10": dict(model="MyActuator RMD-X10-P20-200", mass=1.64, cyl=(0.124, 0.0635), peak=170.0, rated=60.0,
                vel=80 * 2 * math.pi / 60, mesh="rmd_x10_p20_200"),
    "X8": dict(model="MyActuator RMD-X8-P20-120", mass=1.40, cyl=(0.100, 0.076), peak=120.0, rated=43.0,
               vel=158 * 2 * math.pi / 60, mesh="rmd_x8_p20_120"),
    "X6": dict(model="MyActuator RMD-X6-P20-60", mass=0.82, cyl=(0.080, 0.069), peak=60.0, rated=20.0,
               vel=176 * 2 * math.pi / 60, mesh="rmd_x6_p20_60"),
    "X4": dict(model="MyActuator RMD-X4-P36-36", mass=0.36, cyl=(0.058, 0.064), peak=34.0, rated=10.5,
               vel=111 * 2 * math.pi / 60, mesh="rmd_x4_p36_36"),
    "XH540": dict(model="ROBOTIS Dynamixel XH540-W270", mass=0.165, box=(0.0335, 0.0585, 0.044), peak=10.6,
                  rated=4.0, vel=46 * 2 * math.pi / 60, mesh="dxl_xh540"),
    "XM430": dict(model="ROBOTIS Dynamixel XM430-W350", mass=0.082, box=(0.0285, 0.0465, 0.034), peak=4.1,
                  rated=1.5, vel=46 * 2 * math.pi / 60, mesh="dxl_xm430"),
}

COLORS = {
    "shell": "0.92 0.93 0.95 1", "frame": "0.25 0.27 0.30 1", "actuator": "0.10 0.10 0.11 1",
    "dxl": "0.15 0.15 0.15 1", "sensor": "0.05 0.05 0.06 1", "accent": "0.95 0.45 0.10 1",
    "hand": "0.80 0.80 0.82 1",
}

# structural mass targets (kg) - aluminium 7075 / CF frame + shells + wiring
STRUCT_MASS = {
    "pelvis": 1.10, "torso": 2.20, "hip_yaw_link": 0.15, "hip_roll_link": 0.20, "thigh": 0.75,
    "shank": 0.55, "ankle_pitch_link": 0.08, "foot": 0.40, "shoulder_pitch_link": 0.12,
    "shoulder_roll_link": 0.25, "shoulder_yaw_link": 0.25, "elbow_link": 0.15, "wrist_yaw_link": 0.15,
    "wrist_pitch_link": 0.03, "wrist_roll_link": 0.06, "neck_link": 0.05, "head": 0.40,
}
STRUCT_COLOR = {"pelvis": "shell", "torso": "shell", "thigh": "shell", "shank": "shell", "head": "shell",
                "foot": "frame", "shoulder_roll_link": "shell", "shoulder_yaw_link": "shell",
                "elbow_link": "shell", "wrist_yaw_link": "shell"}

HAND_TOTAL_MASS = 0.54  # Inspire RH56DFQ datasheet; the open model carries 0.19 kg


# ------------------------------------------------------------------ maths
def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(r, p, y):
    return rot("z", y) @ rot("y", p) @ rot("x", r)


def R_to_rpy(R):
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r, y = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return r, p, y


AXIS_ROT = {"x": rot("y", math.pi / 2), "y": rot("x", -math.pi / 2), "z": np.eye(3)}  # maps local z -> axis


class Body:
    """Rigid body: mass, centre of mass, inertia about COM (link frame)."""

    def __init__(self, m=0.0, c=(0, 0, 0), I=None):
        self.m, self.c, self.I = m, np.array(c, float), np.zeros((3, 3)) if I is None else np.array(I, float)

    def __add__(self, o):
        m = self.m + o.m
        if m == 0:
            return Body()
        c = (self.m * self.c + o.m * o.c) / m
        I = np.zeros((3, 3))
        for b in (self, o):
            d = b.c - c
            I += b.I + b.m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        return Body(m, c, I)


def box_body(m, size, c, R=np.eye(3)):
    x, y, z = size
    I = m / 12 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])
    return Body(m, c, R @ I @ R.T)


def cyl_body(m, d, h, c, axis="z"):
    r = d / 2
    I = np.diag([m * (3 * r * r + h * h) / 12] * 2 + [m * r * r / 2])
    R = AXIS_ROT[axis]
    return Body(m, c, R @ I @ R.T)


def mesh_body(mesh, m):
    mp = mesh.copy()
    mp.density = m / mp.volume
    return Body(m, mp.center_mass, mp.moment_inertia)


# ------------------------------------------------------------------ model
class Link:
    def __init__(self, name, part=None):
        self.name, self.part = name, part
        self.body = Body()
        self.visuals, self.collisions = [], []
        self.ledger = []  # (category, item, mass)
        if part:
            mesh = trimesh.load(str(DESC / "meshes" / "structure" / f"{part}.stl"), force="mesh")
            m = STRUCT_MASS[part]
            self.body = self.body + mesh_body(mesh, m)
            self.visuals.append(dict(geom=f'<mesh filename="{PKG}/meshes/structure/{part}.stl"/>',
                                     color=STRUCT_COLOR.get(part, "frame")))
            self.ledger.append(("structure", part, m))

    def actuator(self, key, xyz, axis):
        a = ACT[key]
        R = AXIS_ROT[axis]
        rpy = "%.6f %.6f %.6f" % R_to_rpy(R)
        if "cyl" in a:
            d, h = a["cyl"]
            self.body = self.body + cyl_body(a["mass"], d, h, xyz, axis)
            prim = f'<cylinder radius="{d / 2:.4f}" length="{h:.4f}"/>'
        else:
            bx = a["box"]
            self.body = self.body + box_body(a["mass"], bx, xyz, R)
            prim = f'<box size="{bx[0]:.4f} {bx[1]:.4f} {bx[2]:.4f}"/>'
        self.visuals.append(dict(geom=prim, xyz=xyz, rpy=rpy, color="actuator" if "cyl" in a else "dxl",
                                 vendor=a["mesh"]))
        self.ledger.append(("actuators", a["model"], a["mass"]))

    def component(self, label, m, size, xyz, category="electronics", visual=None, color="sensor", rpy="0 0 0"):
        self.body = self.body + box_body(m, size, xyz)
        if visual:
            self.visuals.append(dict(geom=visual, xyz=xyz, rpy=rpy, color=color))
        self.ledger.append((category, label, m))

    def collide_box(self, size, xyz, rpy="0 0 0"):
        self.collisions.append((f'<box size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"/>', xyz, rpy))


class Joint:
    def __init__(self, name, parent, child, xyz, axis=None, lim=None, act=None, jtype="revolute", rpy=(0, 0, 0),
                 mimic=None, effort=None, vel=None, group=None):
        self.__dict__.update(locals())
        del self.__dict__["self"]


links, joints = {}, []


def L(name, part=None):
    links[name] = Link(name, part)
    return links[name]


def J(*a, **k):
    joints.append(Joint(*a, **k))
    return joints[-1]


def mirror_lim(lim, s):
    return lim if s > 0 else (-lim[1], -lim[0])


def build_body():
    pel = L("pelvis", "pelvis")
    pel.component("Intel NUC (locomotion computer)", 0.50, (0.10, 0.10, 0.035), (-0.02, 0, -0.01))
    pel.component("VectorNav VN-100 IMU", 0.016, (0.036, 0.033, 0.009), (0, 0, 0))
    pel.component("CAN-FD interfaces + lower PDB", 0.20, (0.08, 0.06, 0.03), (0.03, 0, -0.01))
    pel.actuator("X8", (0, 0, 0.005), "z")  # waist yaw
    pel.collide_box((0.150, 0.220, 0.090), (0, 0, -0.005))

    tor = L("torso", "torso")
    tor.component("48 V Li-ion battery 13S3P + BMS", 3.10, (0.07, 0.20, 0.16), (-0.045, 0, 0.13), "power")
    tor.component("NVIDIA Jetson AGX Orin 64GB + carrier + heatsink", 0.90, (0.11, 0.11, 0.06), (0.03, 0, 0.20))
    tor.component("Power distribution, DC-DC, E-stop contactor", 0.60, (0.08, 0.15, 0.04), (0.02, 0, 0.06), "power")
    tor.component("Speaker", 0.10, (0.05, 0.05, 0.03), (0.06, 0, 0.10))
    tor.component("Livox Mid-360 lidar", 0.265, (0.065, 0.065, 0.060), (0.095, 0, 0.215), category="sensors",
                  visual="LIDAR")
    tor.actuator("XM430", (0, 0, D.NECK_Z - 0.03), "z")  # neck yaw
    tor.collide_box((0.170, 0.280, 0.250), (0, 0, 0.150))
    J("waist_yaw_joint", "pelvis", "torso", (0, 0, D.WAIST_Z), "z", (-1.5, 1.5), "X8", group="body")

    # ---------------- legs ----------------
    for side, s in (("left", 1), ("right", -1)):
        p = side + "_"
        hy = L(p + "hip_yaw_link", "hip_yaw_link")
        hy.actuator("X8", (-0.045, 0, D.HIP_ROLL_Z), "x")  # hip roll housing
        pel.actuator("X6", (0, s * D.HIP_Y, D.HIP_YAW_Z), "z")  # hip yaw housing
        J(p + "hip_yaw_joint", "pelvis", p + "hip_yaw_link", (0, s * D.HIP_Y, D.HIP_YAW_Z), "z",
          mirror_lim((-0.6, 0.9), s), "X6", group="body")

        hr = L(p + "hip_roll_link", "hip_roll_link")
        hr.actuator("X10", (0, s * 0.050, 0), "y")  # hip pitch housing
        J(p + "hip_roll_joint", p + "hip_yaw_link", p + "hip_roll_link", (0, 0, D.HIP_ROLL_Z), "x",
          mirror_lim((-0.35, 0.8), s), "X8", group="body")

        th = L(p + "thigh", "thigh")
        th.actuator("X10", (0, 0, -0.130), "y")  # knee housing (drives knee via push-rod)
        th.collide_box((0.085, 0.072, 0.26), (0, 0, -0.165))
        J(p + "hip_pitch_joint", p + "hip_roll_link", p + "thigh", (0, 0, 0), "y", (-2.0, 0.6), "X10",
          group="body")

        sh = L(p + "shank", "shank")
        sh.actuator("X6", (-0.020, 0, -0.070), "y")  # ankle actuator A
        sh.actuator("X6", (-0.020, 0, -0.160), "y")  # ankle actuator B
        sh.collide_box((0.065, 0.058, 0.25), (0, 0, -0.150))
        J(p + "knee_joint", p + "thigh", p + "shank", (0, 0, -D.THIGH), "y", (0.0, 2.3), "X10", group="body")

        L(p + "ankle_pitch_link", "ankle_pitch_link")
        # parallel push-rod ankle: two X6 share pitch (sum) and roll (differential)
        J(p + "ankle_pitch_joint", p + "shank", p + "ankle_pitch_link", (0, 0, -D.SHANK), "y", (-0.8, 0.6),
          None, effort=2 * ACT["X6"]["peak"] * 0.9, vel=ACT["X6"]["vel"] * 0.9, group="body")

        ft = L(p + "foot", "foot")
        ft.collide_box((D.FOOT_L, D.FOOT_W, D.FOOT_T), (D.FOOT_X, 0, -D.ANKLE_H + D.FOOT_T / 2))
        J(p + "ankle_roll_joint", p + "ankle_pitch_link", p + "foot", (0, 0, 0), "x",
          mirror_lim((-0.4, 0.4), s), None, effort=ACT["X6"]["peak"] * 0.8, vel=ACT["X6"]["vel"] * 0.9,
          group="body")

    # ---------------- arms ----------------
    for side, s in (("left", 1), ("right", -1)):
        p = side + "_"
        tor.actuator("X6", (0, s * (D.SHOULDER_Y - 0.040), D.SHOULDER_Z), "y")  # shoulder pitch housing
        sp = L(p + "shoulder_pitch_link", "shoulder_pitch_link")
        sp.actuator("X6", (-0.045, 0, 0), "x")  # shoulder roll housing
        J(p + "shoulder_pitch_joint", "torso", p + "shoulder_pitch_link", (0, s * D.SHOULDER_Y, D.SHOULDER_Z),
          "y", (-3.0, 1.0), "X6", group="body")

        sr = L(p + "shoulder_roll_link", "shoulder_roll_link")
        sr.actuator("X4", (0, 0, D.SHOULDER_YAW_Z + 0.035), "z")  # shoulder yaw housing
        sr.collide_box((0.062, 0.062, 0.090), (0, 0, -0.050))
        J(p + "shoulder_roll_joint", p + "shoulder_pitch_link", p + "shoulder_roll_link", (0, 0, 0), "x",
          mirror_lim((-0.3, 2.6), s), "X6", group="body")

        sy = L(p + "shoulder_yaw_link", "shoulder_yaw_link")
        sy.actuator("X6", (0, 0, D.ELBOW_Z), "y")  # elbow housing
        sy.collide_box((0.058, 0.058, 0.100), (0, 0, -0.055))
        J(p + "shoulder_yaw_joint", p + "shoulder_roll_link", p + "shoulder_yaw_link", (0, 0, D.SHOULDER_YAW_Z),
          "z", (-2.0, 2.0), "X4", group="body")

        el = L(p + "elbow_link", "elbow_link")
        el.actuator("XH540", (0, 0, D.WRIST_YAW_Z + 0.030), "z")  # wrist yaw housing
        el.collide_box((0.052, 0.052, 0.090), (0, 0, -0.040))
        J(p + "elbow_joint", p + "shoulder_yaw_link", p + "elbow_link", (0, 0, D.ELBOW_Z), "y", (-2.4, 0.1),
          "X6", group="body")

        wy = L(p + "wrist_yaw_link", "wrist_yaw_link")
        wy.actuator("XH540", (0, 0, D.WRIST_PITCH_Z + 0.075), "y")  # wrist pitch housing
        wy.actuator("XH540", (0, 0, D.WRIST_PITCH_Z + 0.040), "x")  # wrist roll housing (via linkage)
        wy.collide_box((0.046, 0.046, 0.090), (0, 0, -0.050))
        J(p + "wrist_yaw_joint", p + "elbow_link", p + "wrist_yaw_link", (0, 0, D.WRIST_YAW_Z), "z", (-2.0, 2.0),
          "XH540", group="body")

        L(p + "wrist_pitch_link", "wrist_pitch_link")
        J(p + "wrist_pitch_joint", p + "wrist_yaw_link", p + "wrist_pitch_link", (0, 0, D.WRIST_PITCH_Z), "y",
          (-1.0, 1.0), "XH540", group="body")

        L(p + "wrist_roll_link", "wrist_roll_link")
        J(p + "wrist_roll_joint", p + "wrist_pitch_link", p + "wrist_roll_link", (0, 0, 0), "x",
          mirror_lim((-0.8, 0.8), s), "XH540", group="body")

        build_hand(side, s)

    # ---------------- head ----------------
    nk = L("neck_link", "neck_link")
    nk.actuator("XM430", (0, 0, D.HEAD_PITCH_Z), "y")  # head pitch housing
    J("neck_yaw_joint", "torso", "neck_link", (0, 0, D.NECK_Z), "z", (-1.5, 1.5), "XM430", group="body")

    hd = L("head", "head")
    hd.component("Intel RealSense D435i", 0.072, (0.025, 0.090, 0.025), (0.080, 0, 0.055), category="sensors",
                 visual="CAMERA")
    hd.component("ReSpeaker 4-mic array", 0.05, (0.07, 0.07, 0.01), (0, 0, 0.090), category="sensors")
    hd.collide_box((0.140, 0.130, 0.120), (0.005, 0, 0.040))
    J("head_pitch_joint", "neck_link", "head", (0, 0, D.HEAD_PITCH_Z), "y", (-0.5, 0.8), "XM430", group="body")


def build_hand(side, s):
    S = "L" if s > 0 else "R"
    data = json.loads((CAD / "data" / f"inspire_hand_{S}.json").read_text())
    # G1 wrist frame (forearm along +X) -> MIDIBOT wrist frame (forearm along -Z): rotate +90 deg about Y
    g1_mount = (math.pi, 0, -math.pi / 2) if S == "R" else (0, 0, math.pi / 2)
    R = rot("y", math.pi / 2) @ rpy_to_R(*g1_mount)
    base = f"{S}_hand_base_link"
    extra = HAND_TOTAL_MASS - sum(v["mass"] for v in data["links"].values())
    for name, ln in data["links"].items():
        lk = L(name)
        ixx, ixy, ixz, iyy, iyz, izz = ln["inertia"]
        lk.body = Body(ln["mass"], ln["com"], [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        lk.ledger.append(("hands", "Inspire RH56DFQ hand (links)", ln["mass"]))
        lk.visuals.append(dict(geom=f'<mesh filename="{PKG}/meshes/hands/{ln["mesh"]}.stl"/>', color="hand"))
        mesh = trimesh.load(str(DESC / "meshes" / "hands" / f"{ln['mesh']}.stl"), force="mesh")
        lo, hi = mesh.bounds
        shrink = 0.85 if name != base else 0.9
        lk.collide_box(tuple((hi - lo) * shrink), tuple((hi + lo) / 2))
        if name == base:
            lk.body = lk.body + Body(extra, ln["com"], np.eye(3) * 2e-4)
            lk.ledger.append(("hands", "Inspire RH56DFQ hand (6 linear actuators + driver)", extra))
    J(f"{S}_hand_mount", f"{side}_wrist_roll_link", base, (0, 0, D.HAND_MOUNT_Z), jtype="fixed",
      rpy=R_to_rpy(R))
    for j in data["joints"]:
        J(j["name"], j["parent"], j["child"], tuple(j["xyz"]), tuple(j["axis"]), (j["lower"], j["upper"]), None,
          rpy=tuple(j["rpy"]), mimic=j["mimic"], effort=1.5, vel=2.0, group=f"{side}_hand")


# ------------------------------------------------------------------ writers
def f3(v):
    return " ".join("%.6g" % x for x in v)


def write_urdf():
    out = ['<?xml version="1.0"?>',
           "<!-- GENERATED by src/midibot_cad/scripts/generate_urdf.py - do not edit by hand -->",
           '<robot name="midibot" xmlns:xacro="http://www.ros.org/wiki/xacro">',
           '  <xacro:arg name="vendor_meshes" default="false"/>',
           '  <!-- Dynamixel CAD must be downloaded by hand (see COMPONENTS.md), so it has its own switch -->',
           '  <xacro:arg name="dxl_meshes" default="false"/>',
           '  <xacro:arg name="sim" default="true"/>',
           '  <xacro:arg name="sensors" default="true"/>',
           '  <xacro:arg name="controllers_file" default=""/>']
    for c, rgba in COLORS.items():
        out.append(f'  <material name="{c}"><color rgba="{rgba}"/></material>')
    out += ['  <link name="base_link"/>',
            '  <joint name="base_to_pelvis" type="fixed"><parent link="base_link"/><child link="pelvis"/>'
            '<origin xyz="0 0 0" rpy="0 0 0"/></joint>']

    for ln in links.values():
        b = ln.body
        I = b.I
        out.append(f'  <link name="{ln.name}">')
        out.append(f'    <inertial><origin xyz="{f3(b.c)}" rpy="0 0 0"/><mass value="{b.m:.5f}"/>'
                   f'<inertia ixx="{I[0, 0]:.4e}" ixy="{I[0, 1]:.4e}" ixz="{I[0, 2]:.4e}" '
                   f'iyy="{I[1, 1]:.4e}" iyz="{I[1, 2]:.4e}" izz="{I[2, 2]:.4e}"/></inertial>')
        for v in ln.visuals:
            xyz, rpy = f3(v.get("xyz", (0, 0, 0))), v.get("rpy", "0 0 0")
            geom = v["geom"]
            if geom == "LIDAR":
                out.append('    <xacro:if value="$(arg vendor_meshes)">'
                           f'<visual><origin xyz="{xyz}" rpy="0 0 0"/><geometry><mesh filename="{PKG}'
                           '/meshes/vendor/livox_mid360.stl"/></geometry><material name="sensor"/></visual>'
                           '</xacro:if>')
                out.append('    <xacro:unless value="$(arg vendor_meshes)">'
                           f'<visual><origin xyz="{xyz}" rpy="0 0 0"/><geometry><cylinder radius="0.0325" '
                           'length="0.060"/></geometry><material name="sensor"/></visual></xacro:unless>')
                continue
            if geom == "CAMERA":
                out.append(f'    <visual><origin xyz="{xyz}" rpy="1.5708 0 1.5708"/><geometry><mesh filename='
                           f'"{PKG}/meshes/sensors/d435.stl"/></geometry><material name="sensor"/></visual>')
                continue
            vis = (f'<visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geom}</geometry>'
                   f'<material name="{v["color"]}"/></visual>')
            if v.get("vendor"):
                arg = "dxl_meshes" if v["vendor"].startswith("dxl_") else "vendor_meshes"
                vm = (f'<visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry><mesh filename="{PKG}/meshes/vendor/'
                      f'{v["vendor"]}.stl"/></geometry><material name="{v["color"]}"/></visual>')
                out.append(f'    <xacro:if value="$(arg {arg})">{vm}</xacro:if>')
                out.append(f'    <xacro:unless value="$(arg {arg})">{vis}</xacro:unless>')
            else:
                out.append("    " + vis)
        for geom, xyz, rpy in ln.collisions:
            out.append(f'    <collision><origin xyz="{f3(xyz)}" rpy="{rpy}"/><geometry>{geom}</geometry></collision>')
        out.append("  </link>")

    for j in joints:
        out.append(f'  <joint name="{j.name}" type="{j.jtype}">')
        out.append(f'    <parent link="{j.parent}"/><child link="{j.child}"/>'
                   f'<origin xyz="{f3(j.xyz)}" rpy="{f3(j.rpy)}"/>')
        if j.jtype == "revolute":
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}.get(j.axis, j.axis)
            eff = j.effort if j.effort is not None else ACT[j.act]["peak"]
            vel = j.vel if j.vel is not None else ACT[j.act]["vel"]
            out.append(f'    <axis xyz="{f3(ax)}"/><limit lower="{j.lim[0]:.4f}" upper="{j.lim[1]:.4f}" '
                       f'effort="{eff:.2f}" velocity="{vel:.3f}"/>')
            damping = 0.05 if "hand" in (j.group or "") else 0.5
            out.append(f'    <dynamics damping="{damping}" friction="0.0"/>')
            if j.mimic:
                m = j.mimic
                out.append(f'    <mimic joint="{m["joint"]}" multiplier="{m["multiplier"]}" offset="{m["offset"]}"/>')
        out.append("  </joint>")

    # sensor frames
    out += [
        '  <link name="imu_link"/><joint name="imu_joint" type="fixed"><parent link="pelvis"/>'
        '<child link="imu_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>',
        '  <link name="camera_link"/><joint name="camera_joint" type="fixed"><parent link="head"/>'
        '<child link="camera_link"/><origin xyz="0.0925 0 0.055" rpy="0 0 0"/></joint>',
        '  <link name="camera_optical_frame"/><joint name="camera_optical_joint" type="fixed">'
        '<parent link="camera_link"/><child link="camera_optical_frame"/>'
        '<origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/></joint>',
        '  <link name="lidar_link"/><joint name="lidar_joint" type="fixed"><parent link="torso"/>'
        '<child link="lidar_link"/><origin xyz="0.095 0 0.235" rpy="0 0 0"/></joint>',
    ]
    for side in ("left", "right"):
        out.append(f'  <link name="{side}_sole"/><joint name="{side}_sole_joint" type="fixed">'
                   f'<parent link="{side}_foot"/><child link="{side}_sole"/>'
                   f'<origin xyz="0 0 {-D.ANKLE_H}" rpy="0 0 0"/></joint>')

    out.append('  <xacro:if value="$(arg sim)">')
    out.append('    <xacro:include filename="$(find midibot_description)/urdf/midibot.gazebo.xacro"/>')
    out.append('    <xacro:midibot_gazebo controllers_file="$(arg controllers_file)" sensors="$(arg sensors)"/>')
    out.append("  </xacro:if>")
    out.append('  <xacro:include filename="$(find midibot_description)/urdf/midibot.ros2_control.xacro"/>')
    out.append('  <xacro:midibot_ros2_control sim="$(arg sim)"/>')
    out.append("</robot>")
    (DESC / "urdf" / "midibot.urdf.xacro").write_text("\n".join(out) + "\n")


def write_ros2_control():
    rev = [j for j in joints if j.jtype == "revolute"]
    out = ['<?xml version="1.0"?>',
           "<!-- GENERATED by src/midibot_cad/scripts/generate_urdf.py - do not edit by hand -->",
           '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
           '  <xacro:macro name="midibot_ros2_control" params="sim">',
           '    <ros2_control name="midibot_system" type="system">',
           '      <hardware>',
           '        <xacro:if value="${sim}"><plugin>gz_ros2_control/GazeboSimSystem</plugin></xacro:if>',
           '        <xacro:unless value="${sim}"><plugin>mock_components/GenericSystem</plugin></xacro:unless>',
           '      </hardware>']
    for j in rev:
        # Finger linkage joints keep their URDF <mimic> tag (kinematics, MoveIt) but are commanded
        # explicitly: Gazebo's DART engine cannot enforce mimic constraints, so the hand coupler
        # (midibot_control/hand_coupler) sends linkage-consistent targets for all 12 joints per hand.
        mimic_attr = ' mimic="false"' if j.mimic else ""
        out.append(f'      <joint name="{j.name}"{mimic_attr}>'
                   '<command_interface name="position"/><command_interface name="effort"/>'
                   '<state_interface name="position"><param name="initial_value">0.0</param></state_interface>'
                   '<state_interface name="velocity"/><state_interface name="effort"/></joint>')
    out.append('      <sensor name="pelvis_imu"><state_interface name="orientation.x"/>'
               '<state_interface name="orientation.y"/><state_interface name="orientation.z"/>'
               '<state_interface name="orientation.w"/><state_interface name="angular_velocity.x"/>'
               '<state_interface name="angular_velocity.y"/><state_interface name="angular_velocity.z"/>'
               '<state_interface name="linear_acceleration.x"/><state_interface name="linear_acceleration.y"/>'
               '<state_interface name="linear_acceleration.z"/></sensor>')
    out += ["    </ros2_control>", "  </xacro:macro>", "</robot>"]
    (DESC / "urdf" / "midibot.ros2_control.xacro").write_text("\n".join(out) + "\n")

    groups = {}
    for j in rev:
        groups.setdefault(j.group, []).append(j.name)
    # hand coupling table for the hand coupler node
    coupling = {}
    for j in rev:
        if j.mimic:
            coupling.setdefault(j.group, {}).setdefault(j.mimic["joint"], []).append(
                {"joint": j.name, "multiplier": j.mimic["multiplier"], "offset": j.mimic["offset"]})
    (DESC / "config" / "hand_coupling.json").write_text(json.dumps(coupling, indent=1))
    y = ["# GENERATED by src/midibot_cad/scripts/generate_urdf.py - do not edit by hand",
         "controller_manager:", "  ros__parameters:", "    update_rate: 500", "    use_sim_time: true", "",
         "    joint_state_broadcaster:", "      type: joint_state_broadcaster/JointStateBroadcaster",
         "    imu_sensor_broadcaster:", "      type: imu_sensor_broadcaster/IMUSensorBroadcaster"]
    for g in groups:
        y += [f"    {g}_controller:", "      type: joint_trajectory_controller/JointTrajectoryController"]
    y += ["", "imu_sensor_broadcaster:", "  ros__parameters:", "    sensor_name: pelvis_imu",
          "    frame_id: imu_link", ""]
    for g, names in groups.items():
        y += [f"{g}_controller:", "  ros__parameters:", "    joints:"] + [f"      - {n}" for n in names]
        y += ["    command_interfaces: [position]", "    state_interfaces: [position, velocity]",
              "    allow_partial_joints_goal: true", "    open_loop_control: false",
              "    constraints:", "      goal_time: 0.0", "      stopped_velocity_tolerance: 0.0", ""]
    (DESC / "config" / "ros2_controllers.yaml").write_text("\n".join(y))
    return groups


def write_budget(groups):
    cats = {}
    items = {}
    for ln in links.values():
        for cat, item, m in ln.ledger:
            cats[cat] = cats.get(cat, 0) + m
            items[(cat, item)] = items.get((cat, item), [0, 0.0])
            items[(cat, item)][0] += 1
            items[(cat, item)][1] += m
    total = sum(l.body.m for l in links.values())
    actuated = [j for j in joints if j.jtype == "revolute" and not j.mimic]
    n_body = sum(1 for j in actuated if j.group == "body")
    md = ["# MIDIBOT mass budget", "",
          "_Generated by `src/midibot_cad/scripts/generate_urdf.py` from the URDF model._", "",
          f"**Total mass: {total:.2f} kg** · actuated DOF: {len(actuated)} "
          f"({n_body} body + {len(actuated) - n_body} hand; plus 12 coupled finger-linkage joints)", "",
          "| Category | Mass (kg) |", "|---|---|"]
    for c in ("actuators", "hands", "structure", "power", "electronics", "sensors"):
        if c in cats:
            md.append(f"| {c} | {cats[c]:.2f} |")
    md += ["", "## Line items", "", "| Category | Item | Qty | Mass (kg) |", "|---|---|---|---|"]
    for (cat, item), (n, m) in sorted(items.items()):
        md.append(f"| {cat} | {item} | {n} | {m:.3f} |")
    md += ["", "## Actuator assignment", "", "| Joint | Actuator | Peak torque (Nm) | Max speed (rad/s) |",
           "|---|---|---|---|"]
    for j in joints:
        if j.jtype == "revolute" and j.group == "body":
            name = ACT[j.act]["model"] if j.act else "2x RMD-X6-P20-60 (parallel push-rod)"
            eff = j.effort if j.effort is not None else ACT[j.act]["peak"]
            vel = j.vel if j.vel is not None else ACT[j.act]["vel"]
            md.append(f"| {j.name} | {name} | {eff:.0f} | {vel:.1f} |")
    (REPO / "docs" / "MASS_BUDGET.md").write_text("\n".join(md) + "\n")
    return total, cats


if __name__ == "__main__":
    build_body()
    write_urdf()
    groups = write_ros2_control()
    total, cats = write_budget(groups)
    print(f"links: {len(links)}  joints: {len(joints)}  total mass: {total:.2f} kg")
    for c, m in cats.items():
        print(f"  {c:12s} {m:6.2f} kg")
    for g, n in groups.items():
        print(f"  controller {g}: {len(n)} joints")
