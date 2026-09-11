# Gyro integration and rotational deskewing

*Status: stub, scheduled for week 2.*

## Purpose

Integrate gyro measurements between scan times to obtain the rotation of the LiDAR frame at every per-point timestamp, and undistort each scan into the frame at scan start. Also the IMU-propagated initial guess for registration. Inputs: measured IMU rate, gyro bias and noise from scripts/inspect_bag.py.

## Measured inputs (exp14, scripts/inspect_bag.py, 2026-09-11)

- IMU `/alphasense/imu` at 399.2 Hz (period 2.505 ms, std 0.004 ms), gyro bias at rest
  $(1.06, -0.28, -0.04)\times 10^{-3}$ rad/s, per-sample noise $(2.8, 3.8, 3.2)\times 10^{-3}$ rad/s.
- LiDAR `/hesai/pandar` at 10.00 Hz, about 52 000 points per scan, per-point `timestamp`
  float64 absolute seconds, minimum equal to the header stamp, spread 0.100 s per scan.
- Up axis is $-z_I$, so $+z_L$ (doc 00). Gyro rates in $I$ map to $L$ through $R_{IL}^\top$.

## Derivation

## Worked example

## Spec

### Signatures

### Invariants

### Tests
