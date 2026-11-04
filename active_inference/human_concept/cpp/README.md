# human_concept — C++ core (active-inference body-pose estimator)

Eigen-only C++ port of the Python prototype's algorithmic core
(`../python/.../body tracking/python/`). No torch / DSR / Qt — this is the reusable
math layer that will later drop into the full RoboComp/DSR `human_concept` agent as the
`HumanModel` / `HumanFitter`.

## Layout

- `core/body18.h` — BODY_18 keypoint indices, names, skeleton edges.
- `core/human_kinematic_model.{h,cpp}` — 11-DOF forward kinematics → 18×3 keypoints,
  segment-length derivation, joint limits. (port of `human_kinematic_model.py`)
- `core/vfe_inference.{h,cpp}` — Kabsch SVD alignment + precision-weighted
  Gauss–Newton / Laplace belief update. Jacobian via **central finite differences**
  through the FK+Kabsch chain; prior gradients are closed-form. (port of `vfe_inference.py`)

## Build

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build -j
```

## Notes / build constraints

Keep flags clean: **no `-march=native`, no `-DEIGEN_MAX_ALIGN_BYTES=0`** so the core
links cleanly into the libdsr-based agent later (see `../../CLAUDE.md`).

## Not yet done (future, see plan)

- Wrap the core into the full DSR `human_concept` agent (config/instance/model/fitter/
  scene_graph + presence; `person` DSR nodes), mirroring `bottle_concept`/`table_concept`.
- Live input source (ZED SDK vs a media-plane skeleton frame vs DSR).
- Analytic FK Jacobian swap-in; Pinocchio free-flyer reformulation.
- OpenCV / Qt visualization.
