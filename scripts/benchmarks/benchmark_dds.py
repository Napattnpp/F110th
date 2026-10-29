#!/usr/bin/env python3
"""
F1TENTH — Comprehensive ROS 2 DDS & Network Performance Benchmark
==================================================================
Empirically evaluates ROS 2 middleware performance (rmw_cyclonedds_cpp vs rmw_fastrtps_cpp)
under high-frequency autonomous racing workloads:
- Small Payload (1 KB @ 100 Hz): Odometry, IMU, Control Commands (/drive, /odom, /tf)
- Medium Payload (32 KB @ 40 Hz & 100 Hz): Hokuyo UST-10LX LiDAR Range Scans (/scan)
- Large Payload (256 KB @ 50 Hz): Local Costmaps & Segmented Obstacle PointClouds
- Heavy Payload (1024 KB @ 20 Hz): Global OccupancyGrids & Sensor Images

Measures:
1. Message Drop Rate (%)
2. Round-Trip / End-to-End Latency: Min, Mean, Median (p50), p95, p99, Max (ms)
3. Latency Jitter (Standard Deviation in ms)
4. Throughput (MB/s)
5. Kernel Socket Drops (Udp: RcvbufErrors / SndbufErrors from /proc/net/snmp)
"""

import argparse
import json
import os
import struct
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np


def get_udp_snmp_stats() -> Dict[str, int]:
    """Reads Linux kernel SNMP UDP statistics from /proc/net/snmp."""
    stats = {"InDatagrams": 0, "InErrors": 0, "OutDatagrams": 0, "RcvbufErrors": 0, "SndbufErrors": 0}
    try:
        with open("/proc/net/snmp", "r") as f:
            lines = f.readlines()
        for i, line in enumerate(lines):
            if line.startswith("Udp:"):
                headers = line.split()
                if i + 1 < len(lines):
                    values = lines[i + 1].split()
                    for k in stats.keys():
                        if k in headers:
                            idx = headers.index(k)
                            stats[k] = int(values[idx])
                break
    except Exception:
        pass
    return stats


def run_subscriber(topic: str, expected_count: int, output_file: str, qos_profile: str, timeout_sec: float = 30.0):
    """Subscriber child process: listens for benchmark messages and computes latency."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init()

    if qos_profile == "best_effort":
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
    else:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

    class BenchSub(Node):
        def __init__(self):
            super().__init__("dds_bench_sub")
            self.latencies = []
            self.seqs = []
            self.sub = self.create_subscription(PointCloud2, topic, self.callback, qos)

        def callback(self, msg: PointCloud2):
            t_recv = time.time()
            if len(msg.data) >= 16:
                seq, t_send = struct.unpack("!Qd", msg.data[:16])
                latency_ms = (t_recv - t_send) * 1000.0
                self.latencies.append(latency_ms)
                self.seqs.append(seq)

    node = BenchSub()
    start_time = time.time()

    # Spin until all messages received or timeout
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if len(node.latencies) >= expected_count:
            break
        if time.time() - start_time > timeout_sec:
            break

    # Calculate statistics
    received_count = len(node.latencies)
    if received_count > 0:
        latencies = np.array(node.latencies)
        res = {
            "received_count": received_count,
            "min_ms": float(np.min(latencies)),
            "mean_ms": float(np.mean(latencies)),
            "p50_ms": float(np.percentile(latencies, 50)),
            "p95_ms": float(np.percentile(latencies, 95)),
            "p99_ms": float(np.percentile(latencies, 99)),
            "max_ms": float(np.max(latencies)),
            "jitter_ms": float(np.std(latencies)),
            "seqs": node.seqs,
        }
    else:
        res = {
            "received_count": 0,
            "min_ms": 0.0,
            "mean_ms": 0.0,
            "p50_ms": 0.0,
            "p95_ms": 0.0,
            "p99_ms": 0.0,
            "max_ms": 0.0,
            "jitter_ms": 0.0,
            "seqs": [],
        }

    with open(output_file, "w") as f:
        json.dump(res, f)

    node.destroy_node()
    rclpy.shutdown()


def run_publisher(topic: str, msg_size_bytes: int, rate_hz: float, count: int, qos_profile: str):
    """Publisher child process: sends benchmark messages at precise rate."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init()

    if qos_profile == "best_effort":
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
    else:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

    node = Node("dds_bench_pub")
    pub = node.create_publisher(PointCloud2, topic, qos)

    # Let discovery settle
    time.sleep(1.0)

    # Prepare buffer template
    payload = bytearray(max(16, msg_size_bytes))
    msg = PointCloud2()
    period = 1.0 / rate_hz

    # Send messages
    for seq in range(count):
        t_now = time.time()
        struct.pack_into("!Qd", payload, 0, seq, t_now)
        msg.data = bytes(payload)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

        # Precise pacing
        elapsed = time.time() - t_now
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Allow final messages to clear DDS write queue
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


def execute_single_test(
    msg_size_bytes: int,
    rate_hz: float,
    count: int,
    qos: str,
    topic: str = "/f110_bench_topic",
) -> Dict:
    """Executes a single pub/sub test in separate subprocesses and records metrics."""
    tmp_out = f"/tmp/bench_res_{os.getpid()}_{int(time.time()*1000)}.json"
    snmp_before = get_udp_snmp_stats()

    # Launch subscriber in background subprocess
    sub_cmd = [
        sys.executable,
        os.path.abspath(__file__),
        "--mode",
        "sub",
        "--topic",
        topic,
        "--count",
        str(count),
        "--qos",
        qos,
        "--output-json",
        tmp_out,
    ]
    sub_proc = subprocess.Popen(sub_cmd)

    # Small pause to allow subscriber node to register with DDS discovery
    time.sleep(1.0)

    # Launch publisher in foreground subprocess
    pub_cmd = [
        sys.executable,
        os.path.abspath(__file__),
        "--mode",
        "pub",
        "--topic",
        topic,
        "--count",
        str(count),
        "--rate",
        str(rate_hz),
        "--msg-size-bytes",
        str(msg_size_bytes),
        "--qos",
        qos,
    ]
    pub_proc = subprocess.Popen(pub_cmd)
    pub_proc.wait()

    # Wait for subscriber to finish
    sub_proc.wait(timeout=10.0)

    snmp_after = get_udp_snmp_stats()
    rcvbuf_drops = snmp_after.get("RcvbufErrors", 0) - snmp_before.get("RcvbufErrors", 0)
    in_errors = snmp_after.get("InErrors", 0) - snmp_before.get("InErrors", 0)

    if os.path.exists(tmp_out):
        with open(tmp_out, "r") as f:
            sub_res = json.load(f)
        os.remove(tmp_out)
    else:
        sub_res = {
            "received_count": 0,
            "min_ms": 0.0,
            "mean_ms": 0.0,
            "p50_ms": 0.0,
            "p95_ms": 0.0,
            "p99_ms": 0.0,
            "max_ms": 0.0,
            "jitter_ms": 0.0,
        }

    rec = sub_res.get("received_count", 0)
    drop_pct = ((count - rec) / count) * 100.0 if count > 0 else 0.0
    throughput_mb = (rec * msg_size_bytes) / (1024 * 1024 * (count / rate_hz))

    return {
        "msg_size_bytes": msg_size_bytes,
        "rate_hz": rate_hz,
        "sent_count": count,
        "received_count": rec,
        "drop_rate_pct": drop_pct,
        "min_ms": sub_res.get("min_ms", 0.0),
        "mean_ms": sub_res.get("mean_ms", 0.0),
        "p50_ms": sub_res.get("p50_ms", 0.0),
        "p95_ms": sub_res.get("p95_ms", 0.0),
        "p99_ms": sub_res.get("p99_ms", 0.0),
        "max_ms": sub_res.get("max_ms", 0.0),
        "jitter_ms": sub_res.get("jitter_ms", 0.0),
        "throughput_mb_s": throughput_mb,
        "kernel_rcvbuf_errors": rcvbuf_drops,
        "kernel_in_errors": in_errors,
    }


def main():
    parser = argparse.ArgumentParser(description="F1TENTH ROS 2 DDS & Network Performance Benchmark")
    parser.add_argument("--mode", choices=["pub", "sub", "both", "matrix"], default="matrix")
    parser.add_argument("--topic", default="/f110_bench_topic")
    parser.add_argument("--msg-size-bytes", type=int, default=32768)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--count", type=int, default=500)
    parser.add_argument("--qos", choices=["best_effort", "reliable"], default="best_effort")
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    if args.mode == "sub":
        run_subscriber(args.topic, args.count, args.output_json, args.qos)
        return

    if args.mode == "pub":
        run_publisher(args.topic, args.msg_size_bytes, args.rate, args.count, args.qos)
        return

    if args.mode == "both":
        res = execute_single_test(args.msg_size_bytes, args.rate, args.count, args.qos, args.topic)
        print(json.dumps(res, indent=2))
        return

    if args.mode == "matrix":
        rmw = os.environ.get("RMW_IMPLEMENTATION", "unknown")
        cdds_uri = os.environ.get("CYCLONEDDS_URI", "none")
        print("=" * 80)
        print(f" F1TENTH Autonomous Racing DDS Performance Benchmark")
        print(f" Active RMW:      {rmw}")
        print(f" CycloneDDS URI:  {cdds_uri}")
        print("=" * 80)

        # Standard racing test matrix
        scenarios = [
            ("Telemetry / Odom (1 KB @ 100 Hz)", 1024, 100.0, 500, "best_effort"),
            ("Hokuyo LiDAR (32 KB @ 40 Hz)", 32768, 40.0, 400, "best_effort"),
            ("LiDAR High-Rate (32 KB @ 100 Hz)", 32768, 100.0, 500, "best_effort"),
            ("Costmap / PointCloud (256 KB @ 50 Hz)", 262144, 50.0, 250, "best_effort"),
            ("OccupancyGrid Map (1024 KB @ 20 Hz)", 1048576, 20.0, 100, "best_effort"),
        ]

        results = []
        for name, size_b, rate, count, qos in scenarios:
            print(f"\n[RUNNING] {name} ... (Count: {count}, Payload: {size_b/1024:.1f} KB, Rate: {rate} Hz)")
            res = execute_single_test(size_b, rate, count, qos, args.topic)
            res["scenario"] = name
            results.append(res)
            print(
                f"   [DONE] Drop: {res['drop_rate_pct']:.2f}% | Mean: {res['mean_ms']:.2f} ms | "
                f"p99: {res['p99_ms']:.2f} ms | Max: {res['max_ms']:.2f} ms | Jitter: {res['jitter_ms']:.2f} ms | "
                f"Kernel Buffer Drops: {res['kernel_rcvbuf_errors']}"
            )

        print("\n" + "=" * 80)
        print(" SUMMARY RESULTS MATRIX")
        print("=" * 80)
        print(
            f"{'Scenario':<38} | {'Drop %':<7} | {'Mean (ms)':<9} | {'p99 (ms)':<9} | {'Max (ms)':<9} | {'Jitter':<7} | {'Kernel Drops'}"
        )
        print("-" * 105)
        for r in results:
            print(
                f"{r['scenario']:<38} | {r['drop_rate_pct']:<7.2f} | {r['mean_ms']:<9.2f} | {r['p99_ms']:<9.2f} | {r['max_ms']:<9.2f} | {r['jitter_ms']:<7.2f} | {r['kernel_rcvbuf_errors']}"
            )
        print("=" * 105)

        if args.output_json:
            with open(args.output_json, "w") as f:
                json.dump(
                    {
                        "rmw": rmw,
                        "cyclonedds_uri": cdds_uri,
                        "results": results,
                    },
                    f,
                    indent=2,
                )
            print(f"Results saved to {args.output_json}")


if __name__ == "__main__":
    main()

