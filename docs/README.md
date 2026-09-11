# Derivations

The mathematics behind `ranger-lio`, written before the code. Each document derives one
component, ends in a **Spec** block, and the C++ under `src/` is implemented against that
Spec. The unit tests under `tests/` are the tests the Spec names, so a green `ctest` is the
statement "the implementation agrees with the derivation".

## Reading order

| Doc | Topic | Week | Code | Tests |
|---|---|---|---|---|
| [00-notation](00-notation.md) | Frames, transform convention, ⊞/⊟, noise notation, axis sanity check | 1 | `configs/hilti22.yaml`, `scripts/gt_to_tum.py` | `gt_to_tum.py` assertions |
| [01-so3-se3](01-so3-se3.md) | Exp/Log, Jacobians, adjoint, composition `T_W_L = T_W_I T_I_L` | 1 | `include/ranger_lio/so3.hpp`, `se3.hpp` | `tests/test_so3.cpp`, `test_se3.cpp` |
| [02-gyro-integration-and-deskewing](02-gyro-integration-and-deskewing.md) | Gyro integration between scans, per-point rotational deskew | 2 | `imu_buffer.hpp`, `deskew.hpp` | `test_deskew.cpp` |
| [03-registration-residuals-and-jacobians](03-registration-residuals-and-jacobians.md) | Point-to-plane / GICP residual, analytic Jacobians | 2 | `registration.hpp` | `test_registration.cpp` |
| [04-registration-covariance-and-degeneracy](04-registration-covariance-and-degeneracy.md) | Covariance from the Hessian, eigenvalue degeneracy test | 2 | `registration.hpp` | `test_registration.cpp` |
| [05-pose-graph-map-estimation](05-pose-graph-map-estimation.md) | Pose graph as MAP estimation, BetweenFactor, iSAM2 | 3 | `backend.hpp` | `test_backend.cpp` |
| [06-imu-preintegration](06-imu-preintegration.md) | Preintegrated IMU factor (stretch) | 3 | `backend.hpp` | `test_backend.cpp` |

## Document template

Every document has the same four sections:

1. **Purpose**: what the component does and what the rest of the pipeline needs from it.
2. **Derivation**: the mathematics, with every non-obvious step shown.
3. **Worked example**: real numbers from the Hilti-Oxford calibration or data.
4. **Spec**: (a) the function signatures the code must expose, (b) the invariants that
   hold, (c) the unit tests that check them, with tolerances. This block is what the
   implementation is written against.

## Math rendering on GitHub

GitHub renders `$...$` inline and `$$...$$` or fenced ```` ```math ```` blocks for display.
Rules used in these docs:

- Prefer fenced ```` ```math ```` blocks for anything multi-line. They survive underscores
  and pipes that break the `$$` parser inside tables and lists.
- Do not use `\begin{align}` outside a ```` ```math ```` block (`aligned` inside one is fine).
- Inline math inside a table cell: keep it short, avoid `|` and `_` where possible, or move
  the expression out of the table.
- Push early and check the rendering on GitHub itself. A local Markdown preview is not the
  same renderer.

## Ownership

The derivations are the author's own work. The implementation under `src/` is AI-assisted
and written against these Specs; the tests that hold it to the Spec are the author's.
`small_gicp` and `GTSAM` are reused unchanged, and the dataset is Hilti-Oxford 2022.
