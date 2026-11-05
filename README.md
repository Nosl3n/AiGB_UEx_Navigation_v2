![Logo](images/AGB_Logo.png)

# AIGreenBots Project -- Deliverable 4.2

# Individual Research Project #09: Sensor Fusion and Robotic Perception for Agricultural Robots

Cognitive stack for a **Clearpath Husky A300** agricultural robot simulated in **Webots**. The robot localizes
itself with GNSS, IMU and wheel odometry; builds a semantic map of the orchard (trunks, foliage, unmodelled
obstacles); detects and tracks the workers with the ZED stereo camera and the LiDAR; and navigates autonomously
along the corridors between the tree rows.

It is built on **RoboComp** and the shared **DSR** graph (cortex): every concept of the world (the robot, the
field, the trunks, the vegetation, the people, whatever no concept explains) is maintained by an independent agent
that publishes it in the graph, and navigation reads the resulting graph.

![The Husky navigating the apple orchard in Webots](images/01_webots_simulator_navigating.png)

| Perception (ZED + YOLO26) | Controller map while navigating |
|---|---|
| ![ZED segmentation](images/02_zed_yolo_segmentation.png) | ![Controller map](images/03_controller_map_navigating.png) |

**Status:** localization, trunk mapping, people tracking and autonomous navigation work in simulation.

## How it works

```
 Webots (husky_apple.wbt)
   │  webots-bridge: skid-steer base, GPS, IMU, ZED, helios LiDAR (Ice + DDS)
   ▼
 Sensorimotor layer ── lidar3d_dds (helios) · zed_camera · imu_dds
   │
   ▼
 Shared DSR graph ◄──► cognitive-layer agents
   robot_concept        the robot's body and sensors (husky.json)
   openfield_concept    localization: EKF with GNSS + IMU + wheels → field node (ENU field frame)
   retina               YOLO26 on the ZED: people (segmentation and pose) and vegetation (ADE20K)
   human_concept        people: 3D skeleton and tracking with the ZED and the LiDAR
   vegetation_concept   foliage (canopies) → what occludes what
   trunk_concept        trunks: low band of the "tree" class confirmed by the LiDAR
   residual_concept     occupancy grid of whatever no concept explains
   controller           planner and base control
```

| Component | What it does |
|---|---|
| `webots-agri/` | Robot (`HuskyA300.proto`), procedural apple tree (`AppleTree.proto`), tilled ground, the `husky_apple.wbt` world, the `dos_fields.wbt` world (two 5-row fields 20 m apart, no people) and the row generator |
| `webots-bridge/` | Webots → RoboComp bridge: skid-steer base, GPS (WGS84 → local frame), sensors and, optionally, a simulated GNSS degradation |
| `robocomp-robolab/` | Sensor drivers: `lidar3d_dds` (helios), `zed_camera`, `imu_dds`, Xbox gamepad |
| `active_inference/` | DSR cognitive agents (listed above), shared modules (`common/`) and launchers (`utils/`). `row_metaconcept` (tree rows) exists but is disabled |

## 1. Base installation (outside this repository)

Tested on **Ubuntu 24.04** with an NVIDIA GPU.

1. **RoboComp** — <https://github.com/robocomp/robocomp>. Follow its installation guide (it also installs ZeroC
   Ice, which RoboComp uses between components). It must end up in `~/robocomp`, with `core`, `interfaces`, `tools`
   and `cmake`.
2. **cortex (DSR graph)** — <https://github.com/robocomp/cortex> (`development` branch). Install it in `/usr/local`
   with its dependencies (eProsima Fast-DDS and Fast-CDR, Qt6, Eigen, cppitertools, libQGLViewer) and the `pydsr`
   Python module. This project also needs the **`field`** node type: add `REGISTER_NODE_TYPE(field)` to
   `core/include/dsr/core/types/type_checking/dsr_node_type.h` and reinstall cortex.
3. **Webots R2025a** — <https://cyberbotics.com> (or <https://github.com/cyberbotics/webots/releases>), installed in
   `/usr/local/webots`. The first start needs network access to download some PROTOs.
4. **ONNX Runtime 1.30 with CUDA** — <https://github.com/microsoft/onnxruntime/releases>, unpacked in
   `/opt/onnxruntime`.
5. **TensorRT 10** (optional, recommended: ~4× faster than CUDA). It must be version 10, the one ONNX Runtime 1.30
   expects:

   ```bash
   sudo apt install libnvinfer10=10.16.1.11-1+cuda13.2 libnvinfer-plugin10=10.16.1.11-1+cuda13.2 libnvonnxparsers10=10.16.1.11-1+cuda13.2
   sudo apt-mark hold libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10
   ```

6. **Other libraries** (apt): Embree, TBB, Qt6, Eigen, tomlplusplus.

## 2. This repository

```bash
cd ~/robocomp
git clone https://github.com/Nosl3n/AiGB_UEx_Navigation_v2.git      # must end up in ~/robocomp/AiGB_UEx_Navigation_v2
```

The launchers use that path.

**`retina` models** (not versioned). Export them from the official Ultralytics weights and place them in
`active_inference/retina/models/yolo26/`:

```bash
pip install ultralytics
for m in yolo26x-seg yolo26x-pose yolo26x-sem-ade20k; do
  yolo export model=$m.pt format=onnx opset=17 simplify=True end2end=False
done
```

| Model | Use |
|---|---|
| `yolo26x-seg` | person masks → `human_concept` |
| `yolo26x-pose` | 3D person skeletons → `human_concept` |
| `yolo26x-sem-ade20k` | semantic segmentation of vegetation → `trunk_concept`, `vegetation_concept` |

On the first start TensorRT builds its engines (1–2 min) in `active_inference/retina/.trt_cache/`.

**Build.** Each component is built in its own folder, with **at most `-j8`** (more threads can exhaust the
memory):

```bash
cmake -B build && make -C build -j8
```

Components: `webots-bridge`;
`robocomp-robolab/components/hardware/{laser/lidar3d_dds,camera/zed_camera,imu/imu_dds}`;
`active_inference/{robot_concept,openfield_concept,residual_concept,controller,retina,human_concept,vegetation_concept,trunk_concept}`.

## 3. Run

```bash
./apple.bash            # Webots with husky_apple.wbt + sensorimotor layer + cognitive layer
./apple.bash --route    # ... and also drives a fixed route along the corridors
./apple.bash --stop     # stops everything in order: cognitive layer, sensorimotor layer and Webots
./apple_field.bash      # same, in dos_fields.wbt (two fields, no people); also --route, --stop, --dry-run
```

- **Navigation goal:** click on the controller's 2D view (press *Run*), or
  `python3 active_inference/utils/send_goal.py X Y` (metres in the field frame: x east, y north).
- **Xbox gamepad:** forward with the left stick, turn with the right one.
- **Web monitor** of the agents' state: <http://127.0.0.1:8080>.
- Layer-by-layer start-up and launcher details: [`active_inference/utils/README.md`](active_inference/utils/README.md).

## 4. The robot and the orchard

- **Husky A300:** 4 skid-steer wheels; 3D LiDAR `helios` (50 layers, 110° vertical, 30 m); ZED stereo camera; IMU;
  GNSS in WGS84 referenced next to the University of Extremadura campus in Badajoz.
- **Orchard:** 5 rows × 12 tall-spindle apple trees (1.3–3.2 m tall, canopies above the GNSS antenna), 2.5 m
  corridors, tilled ground with raised beds. Two workers walk through the orchard.
- **World frame:** the graph's `field` node (ENU), created by `openfield_concept`.

## 5. Open problems

### Phantom obstacles on uneven ground (robot attitude and terrain are not modelled)

**Symptom** (inspection mode, 2026-11-04). The robot reached a standpoint inside a 1.3 m corridor and could not leave
it: the controller reported `start was NOT footprint-feasible`, its grid search exhausted the whole free space every
cycle (≈ 760 000 expansions, the control loop dropped to 0.8 Hz) and it held the base indefinitely.

**Probable cause: two things the pipeline assumes and the field does not satisfy.**

1. **The robot is assumed to be level.** The IMU publishes roll and pitch, but `openfield_concept` fuses only the yaw,
   and the `robot → world` RT is published as SE(2) (rotation `{0, 0, θ}`). Every consumer (controller,
   `residual_concept`, `plant_concept`) projects the LiDAR through that edge. When the Husky tilts on a bed shoulder
   or tilled ground, the ground ahead is projected higher than it is: at 3° of pitch, a return at 5 m rises ≈ 0.26 m.
2. **The ground is assumed to be the plane z = 0.** The orchard has 0.20 m raised beds under each row
   (`FieldTerrain.proto`, `bedHeight`). The controller treats any return above world z = 0.10 m as an obstacle
   (`UnmodelledScanMinZ`), and `residual_concept` uses a fixed 0.25 m ground band above z = 0 (`FloorZ0`). Any slope,
   bed or tilt produces phantom obstacles, which can close a corridor around the robot.

**General fix (pending).**

- **Publish the full attitude.** `openfield_concept` should write the IMU's roll and pitch into the `robot → world`
  RT, using cortex's Euler convention R = Rx·Ry·Rz. The IMU gives R_world←body = Rz(yaw)·Ry(pitch)·Rx(roll) in body
  axes, and its yaw is the EKF's θ. Every consumer already uses the full transform, so they would all benefit.
- **Infer the ground, do not assume it.** Keep a local elevation map with its uncertainty, estimated from the lowest
  LiDAR returns, and define an obstacle as what stands above the *local* ground rather than above z = 0. This
  generalizes `residual_concept/src/residual_floor_plane.h`, which fits a single plane.
- **Treat "the robot is inside an obstacle" as evidence.** The robot physically occupies those cells, so they are
  free. A planning failure from an infeasible start should trigger an epistemic action (look again from where it
  stands), not a hold.
- **Report the outcome.** If there is still no route after that, the controller should report `Unreachable` to the
  affordance producer instead of holding, and it should not repeat a full grid search every control cycle.

### Pending: reference map and localization without GPS

**State today.** The inspection mode works. `openfield_concept` offers standpoints (`afford_openfield`), the
controller drives to them, and the inspection ends when no standpoint is worth its travel; the `field` node then carries
the outcome (`model_stable`, `model_uncertainty`). In `dos_fields.wbt` it inspected the whole field: 11 standpoints reached and 1 failed, with 0.8 %
of the field's entropy left. The plants, rows, plots and field it finds live only in the graph of the current run.
Nothing is saved, and nothing corrects the pose when the GPS fails.

**Next steps (not implemented).** The full specification is in `NELSON/mapeo.md`.

1. **Create the reference map when the inspection completes.** When `model_stable` is raised, freeze the rows (line
   `n(φ)·p = ρ`, extent, state of both ends, covariance), plus the references needed to fix the position *along* the
   rows (e.g. the trunks at the row ends). Save it as compact, versioned JSON with an atomic write and validation.
   The identity must be the map's own, never a graph node id. Load and validate it in later sessions.
2. **Localize against the map when the GPS degrades or is lost.**
   - Detect rows in each LiDAR sweep in the robot frame. These are relative observations: never use the graph's
     plant/row positions, which were placed with the very pose being corrected.
   - Associate the observed rows with the map rows, using the current pose as prior and declaring ambiguity
     instead of guessing.
   - Correct the same EKF: a row fixes heading and lateral position, and end references fix the along-row position.
   - Test by switching the GPS off from `openfield_concept`'s window and comparing drift with and without the map.

A first version of the map format, the atomic store and their tests was written and set aside on 2026-11-03. It is
not in the repository.

## Origin and license

The code starts from `robocomp/active_inference` (`main` @ `5dc985e`), `robocomp/webots-bridge` (`main` @ `486e233`)
and `robocomp/robocomp-robolab` (`master` @ `1cc4825`), by RoboLab (University of Extremadura), keeping the
components the Husky uses. It is distributed under the **GPL-3.0** license, like the code it starts from
([`LICENSE`](LICENSE)).
