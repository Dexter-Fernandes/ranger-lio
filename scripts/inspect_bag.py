#!/usr/bin/env python3
"""Inspect a Hilti ROS 1 bag before conversion: topics, measured rates, PointCloud2 layout,
per-point time statistics, and a stationary IMU window.

Runs on the raw ROS 1 bag through `rosbags` (no ROS needed). The numbers it prints are the
measured inputs for configs/hilti22.yaml and docs/02-gyro-integration-and-deskewing.md:
do not copy rates or bias magnitudes from the paper when this script can measure them.

Usage: scripts/inspect_bag.py /data/hilti22/rosbags/exp14_basement_2.bag [--scans 20] [--imu-seconds 1.0]
"""
import argparse
import sys
from collections import defaultdict

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

PC2_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def structured_dtype(msg):
    """Build a numpy structured dtype from PointCloud2.fields so any point layout is readable."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        base = PC2_DTYPES[f.datatype]
        formats.append(base if f.count == 1 else f"{f.count}{base}")
        offsets.append(f.offset)
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--scans", type=int, default=20, help="number of scans to analyse")
    ap.add_argument("--imu-seconds", type=float, default=1.0, help="window for the stationary IMU statistics")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS1_NOETIC)
    with Reader(args.bag) as reader:
        print(f"bag: {args.bag}")
        print(f"duration: {reader.duration / 1e9:.1f} s, messages: {reader.message_count}")
        print("\ntopics:")
        lidar_conns, imu_conns = [], []
        for c in reader.connections:
            rate = c.msgcount / (reader.duration / 1e9)
            print(f"  {c.topic:40s} {c.msgtype:32s} {c.msgcount:8d} msgs  {rate:7.1f} Hz")
            if c.msgtype == "sensor_msgs/msg/PointCloud2":
                lidar_conns.append(c)
            elif c.msgtype == "sensor_msgs/msg/Imu":
                imu_conns.append(c)
        if not lidar_conns or not imu_conns:
            sys.exit("no PointCloud2 or Imu topic found")

        # ---- LiDAR: field layout and per-point time statistics ------------------------------
        print(f"\nlidar topic: {lidar_conns[0].topic}")
        n = 0
        header_stamps = []
        time_field = None
        for conn, t_bag, raw in reader.messages(connections=lidar_conns[:1]):
            msg = ts.deserialize_ros1(raw, conn.msgtype)
            if n == 0:
                print("  fields:")
                for f in msg.fields:
                    print(f"    {f.name:12s} offset={f.offset:3d} datatype={f.datatype} ({PC2_DTYPES[f.datatype]}) count={f.count}")
                print(f"  point_step={msg.point_step} width={msg.width} height={msg.height} is_dense={msg.is_dense}")
                cands = [f.name for f in msg.fields if f.name in ("timestamp", "time", "t", "time_offset", "stamp")]
                time_field = cands[0] if cands else None
                print(f"  per-point time field: {time_field}")
            dt = structured_dtype(msg)
            pts = np.frombuffer(msg.data, dtype=dt)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            header_stamps.append(stamp)
            if time_field is not None:
                tf = pts[time_field].astype(np.float64)
                spread = tf.max() - tf.min()
                print(f"  scan {n:3d}: header={stamp:.6f} bag={t_bag/1e9:.6f} n={len(pts):6d} "
                      f"time[min,max]=[{tf.min():.6f},{tf.max():.6f}] spread={spread:.6f} "
                      f"min-header={tf.min()-stamp:+.6f}")
            else:
                print(f"  scan {n:3d}: header={stamp:.6f} n={len(pts):6d} (no per-point time field)")
            n += 1
            if n >= args.scans:
                break
        if len(header_stamps) > 1:
            d = np.diff(header_stamps)
            print(f"  scan period: mean={d.mean()*1e3:.2f} ms  std={d.std()*1e3:.2f} ms  -> {1/d.mean():.2f} Hz")

        # ---- IMU: rate and a stationary window at the start ---------------------------------
        print(f"\nimu topic: {imu_conns[0].topic}")
        stamps, gyro, acc = [], [], []
        for conn, t_bag, raw in reader.messages(connections=imu_conns[:1]):
            msg = ts.deserialize_ros1(raw, conn.msgtype)
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            if stamps[-1] - stamps[0] > args.imu_seconds:
                break
        stamps, gyro, acc = np.array(stamps), np.array(gyro), np.array(acc)
        d = np.diff(stamps)
        print(f"  samples in first {args.imu_seconds:.1f} s: {len(stamps)}  period mean={d.mean()*1e3:.3f} ms "
              f"std={d.std()*1e3:.3f} ms -> {1/d.mean():.1f} Hz")
        print("  first 10 samples (t, gyro xyz [rad/s], acc xyz [m/s^2]):")
        for i in range(min(10, len(stamps))):
            print(f"    {stamps[i]:.6f}  {gyro[i,0]:+.5f} {gyro[i,1]:+.5f} {gyro[i,2]:+.5f}  "
                  f"{acc[i,0]:+.4f} {acc[i,1]:+.4f} {acc[i,2]:+.4f}")
        gn, an = np.linalg.norm(gyro, axis=1), np.linalg.norm(acc, axis=1)
        print(f"  gyro mean [rad/s]: {gyro.mean(0)}  (bias estimate if stationary)")
        print(f"  gyro std  [rad/s]: {gyro.std(0)}   |gyro| mean={gn.mean():.5f} max={gn.max():.5f}")
        print(f"  acc  mean [m/s^2]: {acc.mean(0)}   |acc| mean={an.mean():.4f} std={an.std():.4f}")
        print(f"  acc  std  [m/s^2]: {acc.std(0)}")
        up = np.argmax(np.abs(acc.mean(0)))
        print(f"  dominant specific-force axis: {'xyz'[up]} sign={'+' if acc.mean(0)[up] > 0 else '-'} "
              f"(accelerometer at rest reads +g along the up axis)")


if __name__ == "__main__":
    main()
