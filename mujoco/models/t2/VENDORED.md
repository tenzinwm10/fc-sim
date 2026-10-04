# Booster T2 assets

- Source: [BoosterRobotics/booster_assets](https://github.com/BoosterRobotics/booster_assets), commit `12a97516f57469078707afb587a5f84c1cb110d1`.
- Original `robots/T2/T2_31dof/T2_31dof.xml` SHA-256: `add4e22e3f37c321a8c443d1745a7e870fc01daf9478aa2d4566b1a4a50d1247`.
- License: BSD-3-Clause in `LICENSE.booster_assets`.
- The 37 T2 STL meshes are copied byte for byte. `ball_001.obj` and `ball_001_color.png` come from NUSim's existing K1 field assets.
- Regenerate with `python mujoco/models/t2/vendor_t2.py /path/to/booster_assets` after checking out the pinned commit.

The generated robot retains Booster's masses, inertia, joints, limits, collision geometry, motors and sensor noise. The script removes the demonstration floor and lights, names the free root joint `root`, adds an accelerometer at the existing IMU site, adds a `ready` pose from Booster Deploy's T2 preparation pose, and adds a provisional `head` camera. The RoboCup M-Field and ball geometry/contact parameters are copied from NUSim's middle scene.

The camera pose and 70-degree vertical FOV were borrowed from NUSim's K1 camera. They are placeholders for T2 calibration and must be measured before perception accuracy claims. The `ready` height and contact model must be checked against the real T2 and pitch.

The movement checkpoint used by `tools/t2_policy_bridge.py` is from [BoosterRobotics/booster_deploy](https://github.com/BoosterRobotics/booster_deploy), commit `7bb1462e2ac742eba5c01659b815e2e99fbe617f`, `tasks/locomotion/robots/t2/models/t2_walk.pt`, SHA-256 `acdeefbd6fff460e756f2329a7dc459504750e52c296af618b5292cd1b433aa7`. It is loaded from that checkout, not redistributed here. No public evidence establishes that checkpoint as byte-identical to the robot's built-in WALK/SOCCER policy.
