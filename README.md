# ranger-lio

[![ci](https://github.com/Dexter-Fernandes/ranger-lio/actions/workflows/ci.yml/badge.svg)](https://github.com/Dexter-Fernandes/ranger-lio/actions/workflows/ci.yml)

LiDAR-inertial odometry in C++17 with a GTSAM pose-graph backend and gyro-based deskewing,
built on small_gicp and evaluated on Hilti-Oxford 2022.

**Status: week 1 of 4, infrastructure.** Devcontainer, derivations 00 and 01, SO(3)/SE(3)
primitives with tests, dataset pipeline and evaluation scripts. No odometry yet.

## What this is

An odometry front end (range filter, voxel downsample, gyro-based rotational deskewing at
per-point timestamps, scan-to-local-map GICP with an IMU-propagated initial guess) feeding a
GTSAM pose graph (gauge prior, BetweenFactors with covariances derived from the registration
Hessian, iSAM2). It processes recorded bags offline, scan by scan, on a two-core laptop and
reports scans per second; there is no real-time claim. It is called odometry, not SLAM,
unless loop closure lands.

The mathematics is derived first, in [docs/](docs/README.md). Each derivation ends in a
Spec block, the code is implemented against that Spec, and the unit tests are the tests the
Spec names.

## Ownership

| Derived | Implemented against spec | Reused |
|---|---|---|
| The derivations in `docs/` (frames and conventions, SO(3)/SE(3), gyro integration and deskewing, registration residuals and covariance, pose graph as MAP estimation) and the Spec blocks that define signatures, invariants and tests. Written by the author. | The C++ in `src/` and `include/`, the scripts in `scripts/`, and the devcontainer. AI-assisted, written against the Specs. The tests in `tests/` that hold the implementation to the Spec are the author's; a green `ctest` is the evidence that derivation and implementation agree. | [small_gicp](https://github.com/koide3/small_gicp) (registration, voxel map, Hessian), [GTSAM](https://github.com/borglab/gtsam) (factor graph, iSAM2), [evo](https://github.com/MichaelGrupp/evo) (metrics), [rosbags](https://gitlab.com/ternaris/rosbags) (bag conversion), and the [Hilti-Oxford 2022](https://hilti-challenge.com/dataset-2022.html) dataset, calibration and ground truth. Used unchanged. |

## Dataset and evaluation

Hilti-Oxford 2022 (Hesai PandarXT-32 at 10 Hz, Bosch BMI085 IMU at 400 Hz, hardware
synchronised). Only three sequences have dense 6-DoF ground truth, so those are the three
used; the others carry a handful of sparse 3-DoF control points.

| Role | Sequence | Bag | Duration | Difficulty |
|---|---|---|---|---|
| A, tuning | exp14_basement_2 | 6.3 GB | 73 s | medium: stairs into a basement, door opening |
| B, held out | exp18_corridor_lower_gallery_2 | 8.7 GB | 100 s | hard: long corridor then gallery |
| C, held out | exp16_attic_to_upper_gallery_2 | 15.5 GB | 198 s | hard: narrow staircases, fast motion |

Parameters are tuned on A only, frozen, then B and C are run once. Metrics: ATE and RPE
via evo with SE(3) alignment, fraction of the sequence with a valid pose, scans per second,
peak memory. Ablations: deskewing off, IMU initial guess off. The protocol is fixed in
[results/README.md](results/README.md) before any odometry exists.

## Layout

```
docs/          derivations, one per component, each ending in a Spec block
include/ src/  the ranger_lio library (C++17, no ROS dependency)
tests/         GTest suites named in the Specs
apps/          offline tools (bag reader and runner link rosbag2, never rclcpp)
ros2/          thin ROS 2 Jazzy node, week 4
configs/       dataset and parameter yaml
scripts/       download, bag conversion, bag inspection, ground-truth export, evo evaluation
results/       generated; only the README and final tables are tracked
.devcontainer/ Ubuntu 24.04 + ROS 2 Jazzy image with GTSAM 4.2.2 and small_gicp v1.0.1 pinned
```

## Build

Open the folder in VS Code and reopen in the devcontainer, or build the image directly:

```sh
docker build -f .devcontainer/Dockerfile -t ranger-lio:dev .
docker run --rm -it -v "$PWD":/workspaces/ranger-lio -v /home/dexter/data:/data ranger-lio:dev
```

Inside the container:

```sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/apps/check_deps
```

The first image build compiles GTSAM from source and takes about an hour on a two-core
laptop; later builds hit the cache.

## Week 1 data pipeline

```sh
scripts/download_hilti.sh exp14                 # host: bag, ground truth, calibration -> /home/dexter/data/hilti22
scripts/inspect_bag.py /data/hilti22/rosbags/exp14_basement_2.bag   # container: topics, rates, point layout, IMU window
scripts/convert_bag.sh exp14_basement_2         # container: ROS 1 bag -> rosbag2 mcap, LiDAR + IMU topics only
scripts/gt_to_tum.py --seq exp14_basement_2     # container: gt.tum, identity.tum, gt_lidar_frame.tum + extrinsic assertions
scripts/eval.sh results/exp14 results/exp14/identity.tum identity   # container: evo on identity vs ground truth
```

## Hardware

Developed and evaluated on an Intel i7-7500U (2 cores, 4 threads), 12 GB RAM, CPU only.
Datasets live on the host under `/home/dexter/data` and are bind-mounted; bags are never
copied into the image or the repository.

## Licence

MIT, see [LICENSE](LICENSE).
