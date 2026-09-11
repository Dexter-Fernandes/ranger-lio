#!/usr/bin/env python3
"""Validate a Hilti dense ground-truth file and write the week-1 evaluation inputs.

Writes, for a sequence:
  results/<seq>/gt.tum              ground truth T_W_I as provided (TUM: t x y z qx qy qz qw)
  results/<seq>/identity.tum        identity pose at every ground-truth timestamp
  results/<seq>/gt_lidar_frame.tum  T_W_L = T_W_I * T_I_L   (docs/01-so3-se3.md, "Composition")

Frames and conventions follow docs/00-notation.md: T_A_B maps points expressed in B to A, so
the LiDAR pose in the world is the IMU pose composed with the fixed IMU-to-LiDAR extrinsic.
Quaternions are (x, y, z, w) everywhere in this file, as in the calibration yaml and TUM.

Before touching data the script asserts the extrinsic convention both ways: the calibration
quaternion must map the LiDAR z axis to the IMU -z axis, and T_L_W (T_W_L)^-1 must round-trip
a synthetic point. A wrong quaternion order (w,x,y,z) fails here rather than producing a
plausible-looking wrong trajectory.

Usage: scripts/gt_to_tum.py --config configs/hilti22.yaml --seq exp14_basement_2 [--data-root /data/hilti22]
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot


class SE3:
    """Minimal rigid transform: R (3x3), t (3,). Mirrors ranger_lio::SE3 (docs/01)."""

    def __init__(self, R=None, t=None):
        self.R = np.eye(3) if R is None else np.asarray(R, float)
        self.t = np.zeros(3) if t is None else np.asarray(t, float)

    @classmethod
    def from_xyzw(cls, q_xyzw, t):
        return cls(Rot.from_quat(np.asarray(q_xyzw, float)).as_matrix(), t)

    def compose(self, other):          # T_A_C = T_A_B * T_B_C
        return SE3(self.R @ other.R, self.R @ other.t + self.t)

    def inverse(self):                 # T_B_A = (T_A_B)^-1 = (R^T, -R^T t)
        return SE3(self.R.T, -self.R.T @ self.t)

    def act(self, p):                  # p_A = R p_B + t
        return self.R @ np.asarray(p, float) + self.t

    def xyzw(self):
        return Rot.from_matrix(self.R).as_quat()


def check_extrinsics(T_I_L):
    """Known-answer and round-trip assertions for the calibration convention."""
    # The calibration quaternion (0.7071068, -0.7071068, 0, 0) in (x,y,z,w) is a 180 degree
    # rotation about the axis (1,-1,0)/sqrt(2). It maps z_L -> -z_I, x_L -> -y_I, y_L -> -x_I.
    # Read as (w,x,y,z) it would be a 90 degree rotation about -x, mapping z_L -> +y_I.
    z_L, x_L = np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
    np.testing.assert_allclose(T_I_L.R @ z_L, [0, 0, -1], atol=1e-6, err_msg="quaternion order/convention wrong")
    np.testing.assert_allclose(T_I_L.R @ x_L, [0, -1, 0], atol=1e-6)
    # Round trip through the inverse, both directions.
    p_W = np.array([1.2, -3.4, 5.6])
    T_W_I = SE3.from_xyzw([0.1, 0.2, 0.3, 0.9273618], [10.0, -2.0, 0.5])  # arbitrary, normalised
    T_W_L = T_W_I.compose(T_I_L)
    T_L_W = T_W_L.inverse()
    np.testing.assert_allclose(T_W_L.act(T_L_W.act(p_W)), p_W, atol=1e-9)
    np.testing.assert_allclose(T_L_W.compose(T_W_L).R, np.eye(3), atol=1e-9)
    # Known answer: the LiDAR origin in the world is R_W_I t_I_L + t_W_I.
    np.testing.assert_allclose(T_W_L.t, T_W_I.R @ T_I_L.t + T_W_I.t, atol=1e-12)
    print("extrinsic checks: ok (z_L -> -z_I, x_L -> -y_I, inverse round trip, LiDAR origin)")


def load_tum(path):
    a = np.loadtxt(path, comments="#")
    if a.ndim != 2 or a.shape[1] != 8:
        sys.exit(f"{path}: expected 8 columns (t x y z qx qy qz qw), got shape {a.shape}")
    q = a[:, 4:8]
    norms = np.linalg.norm(q, axis=1)
    if not np.allclose(norms, 1.0, atol=1e-3):
        sys.exit(f"{path}: quaternions not unit norm (min {norms.min():.4f}, max {norms.max():.4f})")
    if np.any(np.diff(a[:, 0]) <= 0):
        sys.exit(f"{path}: timestamps not strictly increasing")
    return a


def write_tum(path, rows):
    np.savetxt(path, rows, fmt="%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/hilti22.yaml")
    ap.add_argument("--seq", default="exp14_basement_2")
    ap.add_argument("--data-root", default=None)
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    ext = cfg["extrinsics"]["T_I_L"]
    T_I_L = SE3.from_xyzw(ext["quaternion_xyzw"], ext["translation_m"])
    check_extrinsics(T_I_L)

    root = args.data_root or cfg["dataset"]["root"]
    gt_rel = cfg["dataset"]["sequences"][args.seq]["ground_truth"]
    gt_path = os.path.join(root, gt_rel)
    gt = load_tum(gt_path)
    out = os.path.join(args.out, args.seq.split("_")[0])
    os.makedirs(out, exist_ok=True)

    span = gt[-1, 0] - gt[0, 0]
    extent = gt[:, 1:4].max(0) - gt[:, 1:4].min(0)
    print(f"{gt_path}: {len(gt)} poses, {span:.1f} s, mean rate {len(gt)/span:.2f} Hz, "
          f"xyz extent {extent.round(2)} m, first t={gt[0,0]:.6f}")

    write_tum(os.path.join(out, "gt.tum"), gt)
    ident = np.zeros_like(gt)
    ident[:, 0] = gt[:, 0]
    ident[:, 7] = 1.0
    write_tum(os.path.join(out, "identity.tum"), ident)

    lidar = np.zeros_like(gt)
    lidar[:, 0] = gt[:, 0]
    for i, row in enumerate(gt):
        T_W_I = SE3.from_xyzw(row[4:8], row[1:4])
        T_W_L = T_W_I.compose(T_I_L)
        lidar[i, 1:4] = T_W_L.t
        lidar[i, 4:8] = T_W_L.xyzw()
    write_tum(os.path.join(out, "gt_lidar_frame.tum"), lidar)
    print(f"wrote {out}/gt.tum, identity.tum, gt_lidar_frame.tum")


if __name__ == "__main__":
    main()
