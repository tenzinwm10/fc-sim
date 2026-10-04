# T2 simulation stack

This fork adds an official Booster T2 31-DoF MuJoCo model to NUSim while retaining the upstream K1 build. The simulator owns physics, camera rendering, DDS state and the single Loco API RPC response. A ROS 2 sidecar runs Booster Deploy's public `t2_walk` checkpoint and publishes low-level joint targets. FC Isar's existing brain publishes its normal high-level `Move` and mode RPCs.

## Pin and build

- NUSim base: `c70591e668f33b5451348c9c724287a3d6f16c7e`.
- Booster Assets and Deploy pins and licenses: [T2 asset provenance](../mujoco/models/t2/VENDORED.md).
- On Ubuntu 22.04 with Docker and an EGL-capable GPU:

```bash
docker build --network host -t fc-sim-build -f mujoco/docker/Dockerfile mujoco
docker run --rm --network host --ipc host --gpus all \
  -v "$PWD:/workspace/NUSim" -w /workspace/NUSim/mujoco \
  fc-sim-build bash -lc 'cmake -S . -B build-t2 -G Ninja -DFCSIM_T2=ON && cmake --build build-t2 -j4 && ctest --test-dir build-t2 --output-on-failure'
```

Run the simulator with the same host network and IPC settings so ROS 2 DDS and the camera shared-memory segment can be seen by processes on the host:

```bash
docker run --rm --network host --ipc host --gpus all \
  -e MUJOCO_GL=egl -v "$PWD:/workspace/NUSim" \
  -w /workspace/NUSim/mujoco fc-sim-build \
  ./build-t2/bin/sim/soccer --headless --game 3 --on-field-positions
```

For a one-robot physics check, omit `--game 3`. For a window, omit `--headless` and configure X11 as in NUSim's existing Docker launcher.

## Connect the public T2 walk policy

On the same Ubuntu host, install/source ROS 2 and the Booster `booster_interface` and `booster_msgs` packages used by FC Isar. Clone Booster Deploy at the pin above. With its Python dependencies installed:

```bash
export PYTHONPATH=/absolute/path/to/booster_deploy:$PYTHONPATH
python3 mujoco/tools/t2_policy_bridge.py --device cuda
python3 mujoco/tools/t2_camera_bridge.py
```

These are separate long-running commands. The policy process subscribes to `LowState` and high-level `Move`, runs the official Booster Deploy policy, and publishes `LowCmd` to `/joint_ctrl` at 50 Hz. It never publishes an RPC response. NUSim accepts WALKING/SOCCER/Move only while a valid 31-joint policy command stream has arrived in the previous 250 ms. Unsupported GetUp, LieDown and VisualKick requests receive nonzero status. If the stream stops, the joint controller captures and holds the current pose. The policy command also expires to zero velocity after 300 ms without a fresh Move request.

The camera process reads NUSim's shared-memory RGB frame and publishes `/boostercamera/head/rgb`, `/boostercamera/head/rgb/camera_info` and `/head_pose_stamped` for FC Isar. It requires the same IPC namespace as NUSim. T2 camera extrinsics and intrinsics remain provisional.

## Validation performed

- MuJoCo loads the T2 soccer scene with 31 actuators, ball, head camera and IMU; `test_t2_model` checks this contract.
- The T2 executable builds in Docker and starts a six-robot 3v3 scene (186 actuators). Only the primary T2 receives DDS control at present; additional robots are stationary, pose-held collision bodies.
- A single T2 in PREPARE remains standing in a free-run headless simulation for over 100 simulated seconds.
- `python mujoco/test/t2_walk_closed_loop.py --booster-deploy /path/to/booster_deploy --seconds 5 --vx 0.3` runs the pinned `t2_walk` checkpoint in this exact NUSim scene. It travelled 1.19 m with minimum base height 0.962 m in the local smoke test.

## Required acceptance before using this for robot decisions

1. On the RTX host, verify EGL image publication, camera frame rate, timestamps and geometry. Record the T2 camera resolution, FOV, optical transform, exposure, latency and noise from the robot and replace the provisional MJCF camera calibration.
2. Source FC Isar's actual ROS environment. Run its brain and vision unchanged against this simulator. Capture a ROS bag for `Move` request/response, `LowState`, odometry, head pose, image and camera info. Confirm exactly one RPC response per UUID and that requested body velocity produces physical motion. Compare message definitions and QoS with the installed T2 SDK.
3. Measure turf friction, ball bounce/rolling, motor latency, IMU bias/noise, odometry drift and camera latency on the T2 and venue surface. Tune simulator parameters from recorded measurements, with a held-out replay to avoid fitting only one run.
4. Add real movement policies for get-up and kick or explicitly gate those FC Isar behaviours in simulation. `t2_walk` alone cannot validate full soccer gameplay.
5. Give each additional robot its own control/observation channel and policy or team software. NUSim's current `--game 3` supplies six physical bodies and GameController state, but five are pose-held. Add multiple robot identities, independent DDS namespaces/domains, camera streams and per-agent logs, then run 3v3 closed-loop matches.

The public `t2_walk` checkpoint has not been established as the exact factory firmware WALK/SOCCER controller. This stack tests the public checkpoint and the high-level FC Isar control path; it does not prove byte-level factory-policy parity.
