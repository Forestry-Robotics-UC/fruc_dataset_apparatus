#!/usr/bin/env python3
"""Lightweight ROS 2 window probe for counts, pairability, and stamp alignment."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import time
from typing import Dict, List, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def parse_pair(value: str) -> Tuple[str, str]:
    if ":" not in value:
        raise argparse.ArgumentTypeError(f"expected '<left>:<right>', got '{value}'")
    left, right = value.split(":", 1)
    left = left.strip()
    right = right.strip()
    if not left or not right:
        raise argparse.ArgumentTypeError(f"invalid pair '{value}'")
    return left, right


def ns_from_stamp(msg) -> int | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    try:
        return int(sec) * 1_000_000_000 + int(nanosec)
    except Exception:
        return None


def percentile(sorted_vals: List[float], p: float) -> float | None:
    if not sorted_vals:
        return None
    rank = int(math.ceil((p / 100.0) * len(sorted_vals))) - 1
    rank = max(0, min(rank, len(sorted_vals) - 1))
    return sorted_vals[rank]


def nearest_deltas_ms(reference_ns: List[int], query_ns: List[int]) -> List[float]:
    if not reference_ns or not query_ns:
        return []
    reference_ns = sorted(reference_ns)
    deltas_ms: List[float] = []
    for q in sorted(query_ns):
        idx = bisect.bisect_left(reference_ns, q)
        candidates = []
        if idx > 0:
            candidates.append(abs(q - reference_ns[idx - 1]))
        if idx < len(reference_ns):
            candidates.append(abs(q - reference_ns[idx]))
        if candidates:
            deltas_ms.append(min(candidates) / 1_000_000.0)
    return deltas_ms


class WindowProbe(Node):
    def __init__(
        self,
        duration_sec: float,
        count_topics: List[str],
        stamp_topics: List[str],
        pair_topics: List[Tuple[str, str]],
        align_topics: List[Tuple[str, str]],
        discovery_wait_sec: float,
    ) -> None:
        super().__init__("diag_window_topic_probe")
        self.duration_sec = duration_sec
        self.discovery_wait_sec = discovery_wait_sec

        self.count_topics = list(dict.fromkeys(count_topics))
        self.stamp_topics: Set[str] = set(stamp_topics)
        self.pair_topics = pair_topics
        self.align_topics = align_topics

        self.counts: Dict[str, int] = {t: 0 for t in self.count_topics}
        self.stamps_ns: Dict[str, List[int]] = {t: [] for t in self.stamp_topics}
        self.topic_types: Dict[str, str] = {}
        self.missing_topics: List[str] = []
        self.type_errors: Dict[str, str] = {}
        self._subscriptions = []
        # Best-effort keeps this probe read-only and low overhead for live streams.
        self.subscription_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _topic_type_map(self) -> Dict[str, str]:
        result: Dict[str, str] = {}
        for topic_name, types in self.get_topic_names_and_types():
            if types:
                result[topic_name] = types[0]
        return result

    def _build_cb(self, topic: str):
        def _cb(msg) -> None:
            self.counts[topic] = self.counts.get(topic, 0) + 1
            if topic in self.stamp_topics:
                stamp_ns = ns_from_stamp(msg)
                if stamp_ns is not None:
                    self.stamps_ns.setdefault(topic, []).append(stamp_ns)

        return _cb

    def setup_subscriptions(self) -> None:
        unresolved = set(self.count_topics)
        deadline = time.monotonic() + max(0.0, self.discovery_wait_sec)

        while unresolved and time.monotonic() < deadline:
            type_map = self._topic_type_map()
            resolved_this_round = []
            for topic in sorted(unresolved):
                type_name = type_map.get(topic)
                if not type_name:
                    continue
                self.topic_types[topic] = type_name
                try:
                    msg_type = get_message(type_name)
                except Exception as exc:
                    self.type_errors[topic] = str(exc)
                    resolved_this_round.append(topic)
                    continue
                self._subscriptions.append(
                    self.create_subscription(msg_type, topic, self._build_cb(topic), self.subscription_qos)
                )
                resolved_this_round.append(topic)
            for topic in resolved_this_round:
                unresolved.discard(topic)
            rclpy.spin_once(self, timeout_sec=0.1)

        for topic in sorted(unresolved):
            self.missing_topics.append(topic)

    def run(self) -> Dict[str, object]:
        self.setup_subscriptions()
        end_time = time.monotonic() + max(0.0, self.duration_sec)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        pairability: Dict[str, Dict[str, object]] = {}
        for image_topic, meta_topic in self.pair_topics:
            image_count = int(self.counts.get(image_topic, 0))
            metadata_count = int(self.counts.get(meta_topic, 0))
            denom = max(image_count, metadata_count, 1)
            mismatch_rate = abs(image_count - metadata_count) / float(denom)
            pairability[f"{image_topic}::{meta_topic}"] = {
                "image_count": image_count,
                "metadata_count": metadata_count,
                "mismatch_rate": mismatch_rate,
            }

        alignment: Dict[str, Dict[str, object]] = {}
        for lidar_topic, camera_topic in self.align_topics:
            lidar_stamps = self.stamps_ns.get(lidar_topic, [])
            camera_stamps = self.stamps_ns.get(camera_topic, [])
            deltas_ms = nearest_deltas_ms(lidar_stamps, camera_stamps)
            key = f"{lidar_topic}::{camera_topic}"
            if not lidar_stamps:
                alignment[key] = {"status": "missing_lidar_stamps", "count": 0}
                continue
            if not camera_stamps:
                alignment[key] = {"status": "missing_camera_stamps", "count": 0}
                continue
            if not deltas_ms:
                alignment[key] = {"status": "no_deltas", "count": 0}
                continue
            sorted_deltas = sorted(deltas_ms)
            alignment[key] = {
                "status": "ok",
                "count": len(sorted_deltas),
                "p50_ms": percentile(sorted_deltas, 50.0),
                "p95_ms": percentile(sorted_deltas, 95.0),
                "p99_ms": percentile(sorted_deltas, 99.0),
                "max_ms": sorted_deltas[-1],
            }

        return {
            "status": "ok",
            "duration_sec": self.duration_sec,
            "count_topics": self.count_topics,
            "stamp_topics": sorted(self.stamp_topics),
            "topic_types": self.topic_types,
            "missing_topics": self.missing_topics,
            "type_errors": self.type_errors,
            "counts": self.counts,
            "stamp_counts": {k: len(v) for k, v in self.stamps_ns.items()},
            "pairability": pairability,
            "alignment": alignment,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS2 diagnostics window probe")
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--discovery-wait-sec", type=float, default=1.5)
    parser.add_argument("--count-topic", action="append", default=[])
    parser.add_argument("--stamp-topic", action="append", default=[])
    parser.add_argument("--pair-topic", action="append", type=parse_pair, default=[])
    parser.add_argument("--align-topic", action="append", type=parse_pair, default=[])
    args = parser.parse_args()

    rclpy.init(args=None)
    node = WindowProbe(
        duration_sec=float(args.duration_sec),
        count_topics=[str(t) for t in args.count_topic],
        stamp_topics=[str(t) for t in args.stamp_topic],
        pair_topics=list(args.pair_topic),
        align_topics=list(args.align_topic),
        discovery_wait_sec=float(args.discovery_wait_sec),
    )
    try:
        result = node.run()
        print(json.dumps(result, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
