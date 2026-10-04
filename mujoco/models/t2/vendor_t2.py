"""Rebuild the T2 robot and soccer scene from pinned Booster and NUSim assets.

Usage: python vendor_t2.py /path/to/booster_assets
The input checkout must be at the commit recorded in VENDORED.md.
"""

from __future__ import annotations

import shutil
import sys
import re
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path


HERE = Path(__file__).resolve().parent
K1 = HERE.parent / "k1"
T2_REL = Path("robots/T2/T2_31dof")
PINNED_MODEL_SHA256 = "add4e22e3f37c321a8c443d1745a7e870fc01daf9478aa2d4566b1a4a50d1247"


def main(assets: Path) -> None:
    source = assets / T2_REL
    actual_hash = hashlib.sha256((source / "T2_31dof.xml").read_bytes()).hexdigest()
    if actual_hash != PINNED_MODEL_SHA256:
        raise ValueError(f"Booster T2 model hash changed: {actual_hash}")
    original = ET.parse(source / "T2_31dof.xml")
    robot = original.getroot()
    robot.set("model", "T2_31dof_NUSim")

    # Preserve Booster's physical body, inertial, collision and actuator data.
    # Move only its demonstration floor/lighting into the soccer scene.
    robot.find("worldbody").remove(robot.find("worldbody/geom[@name='floor']"))
    for light in robot.findall("worldbody/light"):
        robot.find("worldbody").remove(light)
    asset = robot.find("asset")
    for element in list(asset):
        if element.tag in {"texture", "material"}:
            asset.remove(element)

    root_joint = robot.find("worldbody/body/joint[@name='world_joint']")
    root_joint.set("name", "root")
    sensors = robot.find("sensor")
    ET.SubElement(sensors, "accelerometer", name="acceleration", site="imu", noise="0.01")
    head = robot.find(".//body[@name='head_pitch_link']")
    ET.SubElement(
        head,
        "camera",
        name="head",
        pos="0.075 0 0.089",
        xyaxes="0 -1 0  0 0 1",
        fovy="70",
    )

    # Booster Deploy T2_31DOF_CFG.prepare_state.joint_pos in SDK serial order.
    ready_by_name = {
        "left_shoulder_pitch_joint": 0.2,
        "left_shoulder_roll_joint": -1.3,
        "left_elbow_yaw_joint": -0.5,
        "right_shoulder_pitch_joint": 0.2,
        "right_shoulder_roll_joint": 1.3,
        "right_elbow_yaw_joint": 0.5,
        "left_knee_pitch_joint": 0.23,
        "left_ankle_pitch_joint": -0.18,
        "right_knee_pitch_joint": 0.23,
        "right_ankle_pitch_joint": -0.18,
    }
    joints = [j.attrib["name"] for j in robot.findall(".//joint") if j.get("type") != "free"]
    if len(joints) != 31:
        raise ValueError(f"expected 31 T2 joints, found {len(joints)}")
    # Joint tree order is MuJoCo qpos order; it differs from SDK serial order.
    ready_qpos = [0.0, 0.0, 0.979, 1.0, 0.0, 0.0, 0.0]
    ready_qpos += [ready_by_name.get(name, 0.0) for name in joints]
    keyframe = ET.SubElement(robot, "keyframe")
    ET.SubElement(keyframe, "key", name="ready", qpos=" ".join(map(str, ready_qpos)))
    ET.indent(original, space="  ")
    original.write(HERE / "T2_31dof.xml", encoding="unicode", xml_declaration=False)

    meshes = HERE / "meshes"
    meshes.mkdir(exist_ok=True)
    for mesh in source.glob("meshes/*.STL"):
        shutil.copy2(mesh, meshes / mesh.name)
    shutil.copy2(assets / "LICENSE", HERE / "LICENSE.booster_assets")
    shutil.copy2(K1 / "meshes/ball_001.obj", meshes / "ball_001.obj")
    shutil.copy2(K1 / "meshes/ball_001_color.png", meshes / "ball_001_color.png")

    # Reuse NUSim's RoboCup M-Field, goal and ball physics exactly. Only the
    # robot include and robot keyframes change here.
    # Upstream scene comments contain CLI `--flags`, which are accepted by
    # MuJoCo's parser but illegal inside XML comments for ElementTree.
    scene_text = (K1 / "k1_scene_robocup_middle.xml").read_text()
    scene_text = re.sub(r"<!--.*?-->", "", scene_text, flags=re.DOTALL)
    scene = ET.ElementTree(ET.fromstring(scene_text))
    world = scene.getroot()
    world.set("model", "t2_robocup_middle_scene")
    world.find("include").set("file", "T2_31dof.xml")
    old_keys = world.find("keyframe")
    world.remove(old_keys)
    keys = ET.SubElement(world, "keyframe")
    for name, x, y in (("kickoff", -3.0, 0.0), ("own_bottom_left", -6.0, -4.0)):
        pose = ready_qpos.copy()
        pose[0], pose[1] = x, y
        ET.SubElement(keys, "key", name=name, qpos=" ".join(map(str, pose)))
    # The ball free joint is zero-padded by MuJoCo, as in NUSim's K1 scene.
    # Its visual/collision geoms are offset from that zero body pose.
    ET.indent(scene, space="  ")
    scene.write(HERE / "t2_scene_robocup_middle.xml", encoding="unicode", xml_declaration=False)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(Path(sys.argv[1]).resolve())
