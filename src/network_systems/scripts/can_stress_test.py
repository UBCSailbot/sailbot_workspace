#!/usr/bin/env python3
"""CAN load soak test: verify ROS keeps up with hardware CAN message rates.

Replays a captured CAN bus log at the exact recorded per-message rate (looping
indefinitely) into can_transceiver, while watching the ROS topics local_pathfinding
depends on (gps, ais_ships, filtered_wind_sensor, ...) for growing delay. Run it for
12hr+ to prove no backlog builds up under sustained load.

How frames are delivered
------------------------
can_transceiver.cpp enables CAN_RAW_FD_FRAMES and always reads a fixed
sizeof(struct canfd_frame) == 72 bytes per frame; classic 16-byte can_frames are
treated as read errors and dropped. So this script always transmits full 72-byte
CAN-FD frames, via one of two channels:

  --iface can0    a real (or vcan) SocketCAN interface. Matches production exactly.
                  Requires CAN hardware or the vcan kernel module.

  (default)       dev-container mode: no CAN hardware needed. In development/sim
                  mode, can_transceiver_ros_intf.cpp binds its CanTransceiver to a
                  plain tmp file from mockCanFd("/tmp/CanSimIntfXXXXXX") instead of
                  a socket, and registerCanCbs() -- the real CAN parsing/publishing
                  code -- is wired up unconditionally regardless of mode. Appending
                  72-byte frames to that file drives the exact same pipeline as a
                  real socket. The file's randomized path is never printed, but the
                  fd is discoverable via /proc/<pid>/fd. If can_transceiver isn't
                  already running, this script launches `ros2 launch network_systems
                  main_launch.py mode:=development` itself and tears it down on exit.

How delay is detected
---------------------
The custom_interfaces messages carry no std_msgs/Header, so there is no send-side
timestamp to diff against. Instead, each topic's wall-clock inter-arrival gap is the
health signal: the replay sends each CAN ID at a fixed rate forever, so a healthy
pipeline shows a flat gap distribution. Per topic, the script learns a baseline gap,
logs every outlier gap (> --gap-warn-multiplier x baseline) as it happens, buckets
gap statistics over time, and at the end fits a linear trend to the bucket means. A
topic fails (WARN_GROWING_DELAY) if its mean gap grew >50% from first to last bucket
with a positive trend -- i.e. a backlog was accumulating.

Replay timing is anchored to absolute per-frame target timestamps computed from a
fixed start time (not accumulated sleeps), so the send rate itself cannot drift over
a 12hr+ run.

Input log formats (auto-detected)
---------------------------------
  1. Bench logger CSV:  <iso timestamp> <elapsed_s> can0  204  [16]  28 58 43 ...
  2. candump -l ASCII:  (1717632481.831869) can0 204#28584336D9BF13782823307530750000

Usage
-----
  # dev container / sandbox, quick smoke test (~30s)
  ./can_stress_test.py --log sample_can_log.csv --duration 30

  # dev container, 12 hour soak
  ./can_stress_test.py --log /path/to/candump.log --duration 43200

  # on the boat / a host with real or vcan can0, against the production-mode graph
  ./can_stress_test.py --log /path/to/candump.log --duration 43200 --iface can0

Exits 0 on PASS, 1 on FAIL. Results (summary.csv, alerts.log, report.json) are
written to --outdir (default /tmp/can_stress_test/<timestamp>).

Note: a representative capture should include GPS (0x70), wind (0x40/0x41), AIS
(0x60) and rudder (0x50) traffic. IDs 0x200-0x2FF are ELEC debug frames and 0x130-
0x13F pressure frames -- can_transceiver drops both ranges before publishing, so
they contribute bus load but no ROS traffic.
"""
from __future__ import annotations

import argparse
import csv
import importlib
import json
import os
import re
import signal
import socket
import statistics
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

# ---------------------------------------------------------------------------
# CAN-FD frame building and delivery
# ---------------------------------------------------------------------------

CANFD_MAX_DLEN = 64
# struct canfd_frame { canid_t can_id; __u8 len; __u8 flags; __u8 __res0; __u8 __res1;
#                      __u8 data[64]; };
_CANFD_FMT = "=IBBBB64s"


def build_canfd_frame(can_id: int, data: bytes) -> bytes:
    """Pack a CAN ID + payload into a 72B struct canfd_frame wire image."""
    if len(data) > CANFD_MAX_DLEN:
        raise ValueError(f"CAN-FD payload of {len(data)}B exceeds max {CANFD_MAX_DLEN}B")
    return struct.pack(_CANFD_FMT, can_id, len(data), 0, 0, 0, data.ljust(CANFD_MAX_DLEN, b"\x00"))


class SocketSink:
    """Sends frames out a real (or vcan) SocketCAN interface as CAN-FD."""

    def __init__(self, iface: str):
        self._sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self._sock.bind((iface,))

    def send(self, can_id: int, data: bytes) -> None:
        self._sock.send(build_canfd_frame(can_id, data))

    def close(self) -> None:
        self._sock.close()


class SimFileSink:
    """Appends frames to can_transceiver's development/sim-mode mock CAN file.

    CanTransceiver::receive() keeps its own file offset and loops on
    read(fd, buf, 72), so appended 72-byte records are consumed exactly like data
    arriving on a real socket.
    """

    def __init__(self, path: str):
        self._f = open(path, "r+b", buffering=0)

    def send(self, can_id: int, data: bytes) -> None:
        self._f.seek(0, os.SEEK_END)
        self._f.write(build_canfd_frame(can_id, data))

    def close(self) -> None:
        self._f.close()


def find_can_sim_file(timeout_s: float = 30.0) -> str:
    """Locate a running can_transceiver's /tmp/CanSimIntf* file via /proc/<pid>/fd."""
    deadline = time.monotonic() + timeout_s
    last_err = "timed out"
    while time.monotonic() < deadline:
        out = subprocess.run(
            ["pgrep", "-f", "install/lib/network_systems/can_transceiver"],
            capture_output=True, text=True,
        ).stdout
        pids = [int(p) for p in out.split()]
        if not pids:
            last_err = "no running can_transceiver process"
        else:
            for pid in pids:
                fd_dir = f"/proc/{pid}/fd"
                try:
                    entries = os.listdir(fd_dir)
                except OSError:
                    continue  # process died between pgrep and here (e.g. a zombie)
                for entry in entries:
                    try:
                        target = os.readlink(os.path.join(fd_dir, entry))
                    except OSError:
                        continue
                    if os.path.basename(target).startswith("CanSimIntf"):
                        return target
            last_err = f"pid(s) {pids} have no CanSimIntf fd -- not in development/sim mode?"
        time.sleep(0.5)
    raise TimeoutError(f"could not find CAN sim file after {timeout_s}s: {last_err}")


# ---------------------------------------------------------------------------
# CAN log parsing
# ---------------------------------------------------------------------------

_CANDUMP_RE = re.compile(
    r"^\((?P<ts>[0-9]+\.[0-9]+)\)\s+\S+\s+"
    r"(?P<id>[0-9A-Fa-f]+)(?P<sep>#{1,2})(?P<data>[0-9A-Fa-f]*)$"
)


@dataclass
class CanEvent:
    delta_s: float  # seconds since the start of the log
    can_id: int
    data: bytes


def _parse_bench_csv(lines: List[str]) -> List[CanEvent]:
    events: List[CanEvent] = []
    start: Optional[float] = None
    for line in lines:
        parts = line.split()
        if len(parts) < 5:
            continue  # blank/malformed row
        try:
            elapsed = float(parts[1])
        except ValueError:
            continue  # header row
        if not re.fullmatch(r"[0-9A-Fa-f]+", parts[3]):
            continue
        byte_tokens = [p for p in parts[5:] if re.fullmatch(r"[0-9A-Fa-f]{2}", p)]
        if start is None:
            start = elapsed
        data = bytes.fromhex("".join(byte_tokens))
        events.append(CanEvent(elapsed - start, int(parts[3], 16), data))
    return events


def _parse_candump_log(lines: List[str]) -> List[CanEvent]:
    events: List[CanEvent] = []
    start: Optional[float] = None
    for line in lines:
        m = _CANDUMP_RE.match(line.strip())
        if not m:
            continue
        ts = float(m.group("ts"))
        data_hex = m.group("data")
        if m.group("sep") == "##" and data_hex:
            data_hex = data_hex[1:]  # nibble after ## is the CAN-FD flags field
        if start is None:
            start = ts
        data = bytes.fromhex(data_hex) if len(data_hex) % 2 == 0 else b""
        events.append(CanEvent(ts - start, int(m.group("id"), 16), data))
    return events


def load_log(path: Path) -> List[CanEvent]:
    lines = path.read_text().splitlines()
    events = _parse_candump_log(lines) or _parse_bench_csv(lines)
    if not events:
        raise ValueError(f"could not parse any CAN events out of {path}")
    events.sort(key=lambda e: e.delta_s)
    return events


def summarize(events: List[CanEvent]) -> str:
    counts: Dict[int, int] = {}
    for e in events:
        counts[e.can_id] = counts.get(e.can_id, 0) + 1
    span = events[-1].delta_s or 1e-9
    lines = [f"Loaded {len(events)} events spanning {span:.3f}s, one loop iteration:"]
    for can_id, count in sorted(counts.items()):
        lines.append(f"  id=0x{can_id:03X}  count={count:6d}  ~{count / span:6.2f} Hz")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Replay thread
# ---------------------------------------------------------------------------

def replay_loop(
    events: List[CanEvent],
    sink,
    stop: threading.Event,
    end_time: float,
    progress_every_s: float = 60.0,
) -> None:
    loop_period = max(events[-1].delta_s, 1e-3)
    start = time.monotonic()
    sent = 0
    last_progress = start
    iteration = 0
    while not stop.is_set() and time.monotonic() < end_time:
        loop_base = start + iteration * loop_period
        for ev in events:
            if stop.is_set():
                break
            now = time.monotonic()
            if now >= end_time:
                break
            target = loop_base + ev.delta_s
            # sleep in <=1s slices so a stop request is honored promptly
            while not stop.is_set():
                remaining = target - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 1.0))
            try:
                sink.send(ev.can_id, ev.data)
                sent += 1
            except OSError as exc:
                print(f"[replay] send failed for id=0x{ev.can_id:03X}: {exc}", file=sys.stderr)
            now = time.monotonic()
            if now - last_progress >= progress_every_s:
                print(
                    f"[replay] t={(now - start) / 3600:.2f}h loop={iteration} frames_sent={sent}",
                    flush=True,
                )
                last_progress = now
        iteration += 1
    sink.close()
    total_s = time.monotonic() - start
    print(f"[replay] stopped after {total_s:.1f}s, {sent} frames sent", flush=True)


# ---------------------------------------------------------------------------
# ROS topic delay monitor
# ---------------------------------------------------------------------------

# topic -> message class name, matching can_transceiver_ros_intf.cpp's publishers.
# gps / ais_ships / filtered_wind_sensor are the ones node_navigate.py subscribes to.
TOPIC_TYPES = {
    "gps": "GPS",
    "wind_sensors": "WindSensors",
    "filtered_wind_sensor": "WindSensor",
    "ais_ships": "AISShips",
    "batteries": "HelperBattery",
    "rudder": "HelperHeading",
}


@dataclass
class TopicStats:
    count: int = 0
    last_recv: Optional[float] = None
    baseline_gap: Optional[float] = None
    baseline_samples: List[float] = field(default_factory=list)
    bucket_gaps: List[float] = field(default_factory=list)
    bucket_means: List[float] = field(default_factory=list)


def _linreg_slope(ys: List[float]) -> float:
    """Least-squares slope of ys against index 0..n-1 (no numpy dependency)."""
    n = len(ys)
    if n < 2:
        return 0.0
    mean_x = (n - 1) / 2
    mean_y = sum(ys) / n
    num = sum((x - mean_x) * (y - mean_y) for x, y in enumerate(ys))
    den = sum((x - mean_x) ** 2 for x in range(n))
    return num / den if den else 0.0


class DelayMonitor(Node):
    def __init__(self, topics: List[str], outdir: Path, bucket_s: float,
                 baseline_n: int, warn_mult: float):
        super().__init__("can_stress_test_monitor")
        self.baseline_n = baseline_n
        self.warn_mult = warn_mult
        self.stats: Dict[str, TopicStats] = {t: TopicStats() for t in topics}
        self.start_time = time.monotonic()

        outdir.mkdir(parents=True, exist_ok=True)
        self.report_path = outdir / "report.json"
        self._summary_f = (outdir / "summary.csv").open("w", newline="")
        self._summary_w = csv.writer(self._summary_f)
        self._summary_w.writerow(
            ["elapsed_s", "topic", "count", "mean_gap_s", "min_gap_s", "max_gap_s", "stddev_gap_s"]
        )
        self._alerts_f = (outdir / "alerts.log").open("w")

        msg_mod = importlib.import_module("custom_interfaces.msg")
        for topic in topics:
            msg_cls = getattr(msg_mod, TOPIC_TYPES[topic])
            self.create_subscription(msg_cls, topic, self._make_cb(topic), 10)
            self.get_logger().info(f"watching topic '{topic}' ({TOPIC_TYPES[topic]})")
        self.create_timer(bucket_s, self._flush_bucket)

    def _make_cb(self, topic: str):
        def _cb(_msg):
            now = time.monotonic()
            st = self.stats[topic]
            st.count += 1
            if st.last_recv is not None:
                gap = now - st.last_recv
                st.bucket_gaps.append(gap)
                if st.baseline_gap is None:
                    st.baseline_samples.append(gap)
                    if len(st.baseline_samples) >= self.baseline_n:
                        st.baseline_gap = statistics.median(st.baseline_samples)
                        self.get_logger().info(
                            f"[{topic}] learned baseline gap = {st.baseline_gap * 1000:.1f} ms"
                        )
                elif gap > st.baseline_gap * self.warn_mult:
                    elapsed_h = (now - self.start_time) / 3600
                    line = (f"t={elapsed_h:.3f}h topic={topic} gap={gap:.3f}s "
                            f"baseline={st.baseline_gap:.3f}s ratio={gap / st.baseline_gap:.1f}x")
                    self._alerts_f.write(line + "\n")
                    self._alerts_f.flush()
                    self.get_logger().warning(f"possible delay: {line}")
            st.last_recv = now
        return _cb

    def _flush_bucket(self) -> None:
        elapsed = time.monotonic() - self.start_time
        for topic, st in self.stats.items():
            if not st.bucket_gaps:
                continue
            mean_g = statistics.fmean(st.bucket_gaps)
            stdev_g = statistics.pstdev(st.bucket_gaps) if len(st.bucket_gaps) > 1 else 0.0
            self._summary_w.writerow(
                [f"{elapsed:.1f}", topic, len(st.bucket_gaps), f"{mean_g:.4f}",
                 f"{min(st.bucket_gaps):.4f}", f"{max(st.bucket_gaps):.4f}", f"{stdev_g:.4f}"]
            )
            st.bucket_means.append(mean_g)
            st.bucket_gaps = []
        self._summary_f.flush()

    def write_final_report(self) -> dict:
        report = {"runtime_s": time.monotonic() - self.start_time, "topics": {}}
        for topic, st in self.stats.items():
            means = st.bucket_means
            slope = _linreg_slope(means)
            first = means[0] if means else None
            last = means[-1] if means else None
            growth_pct = ((last - first) / first * 100.0) if first else 0.0
            if not means:
                verdict = "NO_DATA"
            elif growth_pct > 50.0 and slope > 0:
                verdict = "WARN_GROWING_DELAY"
            else:
                verdict = "OK"
            report["topics"][topic] = {
                "message_count": st.count,
                "baseline_gap_s": st.baseline_gap,
                "bucket_mean_gap_first_s": first,
                "bucket_mean_gap_last_s": last,
                "growth_pct_first_to_last": growth_pct,
                "drift_slope_s_per_bucket": slope,
                "verdict": verdict,
            }
        self.report_path.write_text(json.dumps(report, indent=2))
        return report

    def destroy_node(self) -> bool:
        self._summary_f.close()
        self._alerts_f.close()
        return super().destroy_node()


# ---------------------------------------------------------------------------
# ROS graph management (dev-container mode)
# ---------------------------------------------------------------------------

def launch_network_systems(log_path: Path) -> subprocess.Popen:
    log_f = log_path.open("w")
    return subprocess.Popen(
        ["ros2", "launch", "network_systems", "main_launch.py", "mode:=development"],
        stdout=log_f, stderr=subprocess.STDOUT,
    )


def teardown_ros(proc: Optional[subprocess.Popen]) -> None:
    if proc is not None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    # ros2 launch doesn't reliably forward shutdown to its node processes, and
    # local_transceiver's socat helper (virtual serial port) outlives its parent.
    subprocess.run(
        ["pkill", "-f",
         "install/lib/network_systems/(can_transceiver|remote_transceiver|local_transceiver)"],
        check=False,
    )
    subprocess.run(["pkill", "-f", "socat.*local_transceiver_test_port"], check=False)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--log", required=True, type=Path, help="captured CAN log to replay")
    parser.add_argument(
        "--duration", required=True, type=float,
        help="seconds to run for (43200 = 12h soak test)",
    )
    parser.add_argument(
        "--iface", default=None,
        help="real/vcan SocketCAN interface to send on. Omit to use dev-container "
        "mode (can_transceiver's development-mode sim file; launches "
        "network_systems automatically if not already running)",
    )
    parser.add_argument(
        "--outdir", type=Path,
        default=Path("/tmp/can_stress_test") / time.strftime("%Y%m%d_%H%M%S"),
        help="where to write summary.csv/alerts.log/report.json",
    )
    parser.add_argument(
        "--topics", default=",".join(TOPIC_TYPES),
        help="comma-separated ROS topics to monitor",
    )
    parser.add_argument(
        "--bucket-seconds", type=float, default=None,
        help="stats aggregation bucket (default: duration/20, clamped to [5s, 300s])",
    )
    parser.add_argument(
        "--baseline-samples", type=int, default=50,
        help="messages used to learn each topic's nominal gap",
    )
    parser.add_argument(
        "--gap-warn-multiplier", type=float, default=5.0,
        help="gap/baseline ratio that logs an alert",
    )
    args = parser.parse_args()

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    unknown = [t for t in topics if t not in TOPIC_TYPES]
    if unknown:
        parser.error(f"unknown topic(s) {unknown}; known: {list(TOPIC_TYPES)}")
    bucket_s = args.bucket_seconds or min(max(args.duration / 20, 5.0), 300.0)

    events = load_log(args.log)
    print(summarize(events))
    args.outdir.mkdir(parents=True, exist_ok=True)
    print(f"Results -> {args.outdir}")

    ros_proc: Optional[subprocess.Popen] = None
    try:
        if args.iface:
            sink = SocketSink(args.iface)
            print(f"Sending on SocketCAN interface {args.iface}")
        else:
            try:
                sim_file = find_can_sim_file(timeout_s=2.0)
            except TimeoutError:
                print("can_transceiver not running; launching network_systems (development mode)")
                ros_proc = launch_network_systems(args.outdir / "ros_launch.log")
                sim_file = find_can_sim_file(timeout_s=30.0)
            sink = SimFileSink(sim_file)
            print(f"Sending into dev-mode CAN sim file {sim_file}")

        rclpy.init()
        monitor = DelayMonitor(
            topics, args.outdir, bucket_s, args.baseline_samples, args.gap_warn_multiplier
        )

        stop = threading.Event()
        end_time = time.monotonic() + args.duration
        replay_thread = threading.Thread(
            target=replay_loop, args=(events, sink, stop, end_time), daemon=True
        )
        replay_thread.start()

        try:
            while time.monotonic() < end_time and not stop.is_set():
                rclpy.spin_once(monitor, timeout_sec=0.5)
        except KeyboardInterrupt:
            print("interrupted, shutting down early")
        stop.set()
        replay_thread.join(timeout=10)

        monitor._flush_bucket()
        report = monitor.write_final_report()
        monitor.destroy_node()
        rclpy.try_shutdown()
    finally:
        teardown_ros(ros_proc)

    print(f"\n=== Final report ({monitor.report_path}) ===")
    print(json.dumps(report, indent=2))
    verdicts = {t: info["verdict"] for t, info in report["topics"].items()}
    if "WARN_GROWING_DELAY" in verdicts.values():
        print("RESULT: FAIL -- growing delay detected on: "
              + ", ".join(t for t, v in verdicts.items() if v == "WARN_GROWING_DELAY"))
        return 1
    print("RESULT: PASS -- no topic showed growing delay over the run")
    return 0


if __name__ == "__main__":
    sys.exit(main())
