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

The angle of ray i in scan k is angle_min[k] + i * angle_increment[k].
"""

import argparse
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def read_scans(bag_path, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        yield deserialize_message(data, LaserScan)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="bag directory")
    p.add_argument("-o", "--output", help="output .npz (default: <bag>/scans.npz)")
    p.add_argument("--topic", default="/scan")
    args = p.parse_args()

    scans = list(read_scans(args.bag, args.topic))
    if not scans:
        raise SystemExit(f"No messages on {args.topic} in {args.bag}")

    m = max(len(s.ranges) for s in scans)
    ranges = np.full((len(scans), m), np.nan, dtype=np.float32)
    for k, s in enumerate(scans):
        ranges[k, :len(s.ranges)] = s.ranges

    out = args.output or os.path.join(args.bag, "scans.npz")
    np.savez_compressed(
        out,
        stamp=np.array([s.header.stamp.sec + 1e-9 * s.header.stamp.nanosec for s in scans]),
        angle_min=np.array([s.angle_min for s in scans], dtype=np.float32),
        angle_increment=np.array([s.angle_increment for s in scans], dtype=np.float32),
        n_rays=np.array([len(s.ranges) for s in scans], dtype=np.int32),
        ranges=ranges,
        range_min=np.float32(scans[0].range_min),
        range_max=np.float32(scans[0].range_max),
        scan_time=np.float32(scans[0].scan_time),
    )
    print(f"{len(scans)} scans, up to {m} rays -> {out}")


if __name__ == "__main__":
    main()
