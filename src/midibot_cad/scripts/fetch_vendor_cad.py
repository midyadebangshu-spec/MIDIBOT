#!/usr/bin/env python3
"""Download official vendor CAD (STEP) for MIDIBOT's electrical components and
convert each to a normalized, decimated STL for use as a URDF visual mesh.

Vendor files are NOT committed to the repository (their licenses do not grant
redistribution). Run this script once after cloning; the URDF then picks the
meshes up with `vendor_meshes:=true`.

Normalization: the mesh is converted to metres, centred on its bounding box,
and rotated so that the actuator output axis lies along +Z. For cylindrical
actuators the axis is the bounding-box dimension that differs from the other
two; for Dynamixel servos ("horn" mode) it is the axis of the largest circular
edge of the output horn, and the mesh is centred on that axis. Sensors keep
their native frame and are only centred.

Requires: cadquery, trimesh, fast-simplification  (pip install ...)
Usage:   python3 fetch_vendor_cad.py [--out DIR] [--keep-step DIR]
                                   [--local dxl_xh540=path.stp dxl_xm430=path.stp]
"""
import argparse
import io
import pathlib
import sys
import tempfile
import urllib.request
import zipfile

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE.parents[1] / "midibot_description" / "meshes" / "vendor"

MYACT = "https://www.myactuator.com/_files/archives/"
LIVOX = "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/"
# ROBOTIS only serves CAD to interactive browser downloads: download these by hand from the
# e-Manual pages and pass them with --local (see COMPONENTS.md).
ROBOTIS = "manual:"

# name -> (url, kind, member-suffix-in-zip or None, normalize mode)
COMPONENTS = {
    "rmd_x10_p20_200": (MYACT + "cab28a_42e41abf73914f2dac51dcf8decf200b.zip", "zip", ".STEP", "axis"),
    "rmd_x8_p20_120": (MYACT + "cab28a_2f52d96407934978a1b0e6c98b33a928.zip", "zip", ".STEP", "axis"),
    "rmd_x6_p20_60": (MYACT + "cab28a_4231acdf891b41f3a56bb690f4c1d312.zip", "zip", ".STEP", "axis"),
    "rmd_x4_p36_36": (MYACT + "cab28a_dcb58d860356468d817e06ffe8d248af.zip", "zip", ".STEP", "axis"),
    "livox_mid360": (LIVOX + "Mid360/mid-360-asm.stp", "step", None, "center"),
    # https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/ -> "XM,H,D-540.N101.I101.STP"
    "dxl_xh540": (ROBOTIS + "2084", "step", None, "horn"),
    # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ -> "x-430_idle.stp"
    "dxl_xm430": (ROBOTIS + "158", "step", None, "horn"),
}

TARGET_FACES = 20000


def download(url: str) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "midibot-cad-fetch/1.0"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return r.read()


def step_bytes(name, url, kind, suffix):
    data = download(url)
    if kind == "step":
        return data
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        members = [m for m in z.namelist() if m.upper().endswith(suffix.upper())]
        if not members:
            raise RuntimeError(f"{name}: no {suffix} file in archive")
        return z.read(members[0])


def horn_axis(shape):
    """Axis (unit vector) and centre (metres) of the largest 6-13 mm radius circle."""
    best = None
    for e in shape.edges().vals():
        if e.geomType() != "CIRCLE":
            continue
        r = e.radius()
        if 6.0 <= r <= 13.0 and (best is None or r > best[0]):
            c = e.Center()
            n = e._geomAdaptor().Circle().Axis().Direction()
            best = (r, np.array([c.x, c.y, c.z]) / 1000.0, np.array([n.X(), n.Y(), n.Z()]))
    if best is None:
        raise RuntimeError("no horn circle found")
    return best[1], best[2]


def step_to_mesh(step_path: pathlib.Path, mode="axis"):
    import cadquery as cq
    import trimesh

    shape = cq.importers.importStep(str(step_path))
    horn = horn_axis(shape) if mode == "horn" else None
    with tempfile.TemporaryDirectory() as td:
        stl = pathlib.Path(td) / "m.stl"
        cq.exporters.export(shape, str(stl), tolerance=0.3, angularTolerance=0.3)
        mesh = trimesh.load(str(stl), force="mesh")
    mesh.apply_scale(0.001)  # STEP files are in millimetres
    return mesh, horn


def normalize(mesh, mode, horn=None):
    import trimesh

    if mode == "horn":
        centre, axis = horn
        axis = axis / np.linalg.norm(axis)
        # rotate horn axis onto +Z, then put the axis through the origin (centred along Z)
        z = np.array([0.0, 0.0, 1.0])
        v = np.cross(axis, z)
        if np.linalg.norm(v) < 1e-9:
            R = np.eye(4) if axis[2] > 0 else trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
        else:
            R = trimesh.transformations.rotation_matrix(np.arccos(np.clip(axis @ z, -1, 1)), v)
        mesh.apply_transform(R)
        c = (R[:3, :3] @ centre)
        lo, hi = mesh.bounds
        mesh.apply_translation([-c[0], -c[1], -(lo[2] + hi[2]) / 2.0])
        return mesh

    lo, hi = mesh.bounds
    mesh.apply_translation(-(lo + hi) / 2.0)
    if mode == "axis":
        ext = mesh.extents
        # the axis is the extent that differs most from the other two
        diffs = [abs(ext[i] - np.mean(np.delete(ext, i))) for i in range(3)]
        axis = int(np.argmax(diffs))
        if axis == 0:
            mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
        elif axis == 1:
            mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    return mesh


def decimate(mesh):
    if len(mesh.faces) > TARGET_FACES:
        mesh = mesh.simplify_quadric_decimation(face_count=TARGET_FACES)
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    ap.add_argument("--keep-step", type=pathlib.Path, default=None,
                    help="also save the original STEP files to this directory")
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--local", nargs="*", default=[], metavar="NAME=PATH",
                    help="use a STEP file you downloaded yourself instead of fetching")
    args = ap.parse_args()
    local = dict(item.split("=", 1) for item in args.local)
    args.out.mkdir(parents=True, exist_ok=True)

    failed = []
    for name, (url, kind, suffix, mode) in COMPONENTS.items():
        if args.only and name not in args.only:
            continue
        try:
            if name in local:
                data = pathlib.Path(local[name]).read_bytes()
            elif url.startswith("manual:"):
                print(f"[{name}] skipped: download by hand and pass --local {name}=<file> "
                      "(see COMPONENTS.md)")
                continue
            else:
                print(f"[{name}] downloading", flush=True)
                data = step_bytes(name, url, kind, suffix)
            with tempfile.TemporaryDirectory() as td:
                sp = pathlib.Path(td) / f"{name}.step"
                sp.write_bytes(data)
                if args.keep_step:
                    args.keep_step.mkdir(parents=True, exist_ok=True)
                    (args.keep_step / f"{name}.step").write_bytes(data)
                print(f"[{name}] converting STEP -> mesh", flush=True)
                mesh, horn = step_to_mesh(sp, mode)
                mesh = decimate(normalize(mesh, mode, horn))
            dst = args.out / f"{name}.stl"
            mesh.export(str(dst))
            e = mesh.extents * 1000
            print(f"[{name}] ok: {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm, {len(mesh.faces)} faces -> {dst}")
        except Exception as exc:  # keep going, report at the end
            print(f"[{name}] FAILED: {exc}", file=sys.stderr)
            failed.append(name)
    if failed:
        print("Failed: " + ", ".join(failed), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
