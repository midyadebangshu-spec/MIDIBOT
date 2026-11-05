"""Shared fixtures. Model tests need pinocchio and the generated URDF (set MIDIBOT_URDF, or have
midibot_description installed so xacro can render it); they are skipped otherwise."""
import os
import subprocess
import sys

import pytest

# run the tests straight from the source tree, without installing the package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _urdf_text():
    path = os.environ.get("MIDIBOT_URDF")
    if path and os.path.exists(path):
        return open(path).read()
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("midibot_description")
        out = subprocess.run(["xacro", os.path.join(share, "urdf", "midibot.urdf.xacro"), "sim:=false",
                              "sensors:=false", "vendor_meshes:=false", "dxl_meshes:=false",
                              "controllers_file:=none"], capture_output=True, text=True, check=True)
        return out.stdout
    except Exception:
        return None


@pytest.fixture(scope="session")
def model():
    pin = pytest.importorskip("pinocchio")
    text = _urdf_text()
    if text is None:
        pytest.skip("generated URDF not available (set MIDIBOT_URDF)")
    return pin.buildModelFromXML(text, pin.JointModelFreeFlyer())
