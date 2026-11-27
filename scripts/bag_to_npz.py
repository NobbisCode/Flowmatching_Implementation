#!/usr/bin/env python3
"""Export LaserScan messages from a rosbag (MCAP) to a NumPy .npz file.

    python3 scripts/bag_to_npz.py data/lidar_raw_static            # -> data/lidar_raw_static/scans.npz
    python3 scripts/bag_to_npz.py data/lidar_raw_static -o out.npz --topic /scan

Contents of the .npz (N = number of scans, M = max. rays per scan):
    stamp            (N,)    float64  header time [s]
    angle_min        (N,)    float32  [rad]
    angle_increment  (N,)    float32  [rad]
    n_rays           (N,)    int32    valid rays per scan (raw scans vary!)
    ranges           (N, M)  float32  NaN padded after n_rays; inf = no return
    range_min, range_max, scan_time  float32 scalars (from the first scan)

If the bag contains /odom, the odometry message closest in time to each scan is added:
    odom_x, odom_y, odom_yaw  (N,)  pose of base_link in the odom frame [m, m, rad]
    odom_v, odom_w            (N,)  linear / angular velocity [m/s, rad/s]
    odom_dt                   (N,)  time between scan and the odometry message used [s]

The angle of ray i in scan k is angle_min[k] + i * angle_increment[k].
"""

import argparse
import math
import os

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def read_topic(bag_path, topic, msg_type):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topic not in {t.name for t in reader.get_all_topics_and_types()}:
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        data = reader.read_next()[1]
        yield deserialize_message(data, msg_type)


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def odom_at(scan_t, odoms):
    """Pick the odometry message closest in time to each scan."""
    t = np.array([stamp(o) for o in odoms])
    idx = np.clip(np.searchsorted(t, scan_t), 1, len(t) - 1)
    idx = np.where(np.abs(t[idx - 1] - scan_t) < np.abs(t[idx] - scan_t), idx - 1, idx)
    out = {k: [] for k in ("odom_x", "odom_y", "odom_yaw", "odom_v", "odom_w")}
    for i in idx:
        o = odoms[i]
        q = o.pose.pose.orientation
        out["odom_x"].append(o.pose.pose.position.x)
        out["odom_y"].append(o.pose.pose.position.y)
        out["odom_yaw"].append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        out["odom_v"].append(o.twist.twist.linear.x)
        out["odom_w"].append(o.twist.twist.angular.z)
    out = {k: np.array(v, dtype=np.float32) for k, v in out.items()}
    out["odom_dt"] = (t[idx] - scan_t).astype(np.float32)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="bag directory")
    p.add_argument("-o", "--output", help="output .npz (default: <bag>/scans.npz)")
    p.add_argument("--topic", default="/scan")
    args = p.parse_args()

    scans = list(read_topic(args.bag, args.topic, LaserScan))
    if not scans:
        raise SystemExit(f"No messages on {args.topic} in {args.bag}")

    m = max(len(s.ranges) for s in scans)
    ranges = np.full((len(scans), m), np.nan, dtype=np.float32)
    for k, s in enumerate(scans):
        ranges[k, :len(s.ranges)] = s.ranges

    scan_t = np.array([stamp(s) for s in scans])
    odoms = list(read_topic(args.bag, "/odom", Odometry))
    extra = odom_at(scan_t, odoms) if odoms else {}

    out = args.output or os.path.join(args.bag, "scans.npz")
    np.savez_compressed(
        out,
        stamp=scan_t,
        angle_min=np.array([s.angle_min for s in scans], dtype=np.float32),
        angle_increment=np.array([s.angle_increment for s in scans], dtype=np.float32),
        n_rays=np.array([len(s.ranges) for s in scans], dtype=np.int32),
        ranges=ranges,
        range_min=np.float32(scans[0].range_min),
        range_max=np.float32(scans[0].range_max),
        scan_time=np.float32(scans[0].scan_time),
        **extra,
    )
    print(f"{len(scans)} scans, up to {m} rays{', with odometry' if extra else ''} -> {out}")


if __name__ == "__main__":
    main()
