#!/usr/bin/env python3
import argparse
import importlib
import math
import statistics
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def load_ros_msg_type(type_name: str):
    # Input format: package/msg/Type (example: sensor_msgs/msg/Imu)
    parts = type_name.split("/")
    if len(parts) != 3 or parts[1] != "msg":
        raise ValueError(f"Invalid --msg-type '{type_name}', expected package/msg/Type")
    package, _, msg_name = parts
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, msg_name)


def stamp_to_ns(msg) -> int:
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    sorted_vals = sorted(values)
    rank = (p / 100.0) * (len(sorted_vals) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return sorted_vals[lo]
    w = rank - lo
    return sorted_vals[lo] * (1.0 - w) + sorted_vals[hi] * w


class StampDeltaProbe(Node):
    def __init__(self, topic: str, msg_type, qos: QoSProfile):
        super().__init__("stamp_delta_probe")
        self._last_stamp_ns = None
        self._deltas_ms = []
        self._count = 0
        self.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, msg):
        self._count += 1
        if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
            return
        stamp_ns = stamp_to_ns(msg)
        if self._last_stamp_ns is not None:
            dt_ms = (stamp_ns - self._last_stamp_ns) / 1_000_000.0
            self._deltas_ms.append(dt_ms)
        self._last_stamp_ns = stamp_ns

    def report(self):
        if not self._deltas_ms:
            print("status=no_stamp_deltas")
            print(f"messages_seen={self._count}")
            return
        vals = self._deltas_ms
        print("status=ok")
        print(f"messages_seen={self._count}")
        print(f"deltas_count={len(vals)}")
        print(f"delta_ms_mean={statistics.fmean(vals):.6f}")
        print(f"delta_ms_std={statistics.pstdev(vals):.6f}")
        print(f"delta_ms_min={min(vals):.6f}")
        print(f"delta_ms_p50={percentile(vals, 50):.6f}")
        print(f"delta_ms_p95={percentile(vals, 95):.6f}")
        print(f"delta_ms_p99={percentile(vals, 99):.6f}")
        print(f"delta_ms_max={max(vals):.6f}")


def build_qos(profile_name: str, depth: int) -> QoSProfile:
    if profile_name == "sensor_data":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def main():
    parser = argparse.ArgumentParser(description="Measure header.stamp delta stability.")
    parser.add_argument("--topic", required=True, help="Topic name (example: /imu/data)")
    parser.add_argument(
        "--msg-type",
        required=True,
        help="ROS2 message type package/msg/Type (example: sensor_msgs/msg/Imu)",
    )
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--qos", choices=["sensor_data", "default"], default="sensor_data")
    parser.add_argument("--depth", type=int, default=50)
    args = parser.parse_args()

    msg_type = load_ros_msg_type(args.msg_type)
    qos = build_qos(args.qos, args.depth)

    rclpy.init()
    node = StampDeltaProbe(args.topic, msg_type, qos)
    end = time.time() + max(0.1, args.duration_sec)
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
