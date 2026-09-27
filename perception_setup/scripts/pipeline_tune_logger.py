#!/usr/bin/env python3
"""
Pipeline inspection parameter tuning logger.

Subscribes to RANSAC line output and DVL altitude, writes a structured CSV
to /tmp/pipeline_tune.csv, and on Ctrl+C prints a summary with concrete
parameter recommendations.

Usage (in a separate terminal, while the mission is running):
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/vortex-cv/perception_setup/scripts/pipeline_tune_logger.py
"""
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vortex_msgs.msg import DVLAltitude, LineSegment2DArray

OUTPUT_CSV = "/tmp/pipeline_tune.csv"
OUTPUT_SUMMARY = "/tmp/pipeline_tune_summary.txt"
UPDATE_INTERVAL_MS = 200  # must match line_filtering_params.yaml


class TuningLogger(Node):
    def __init__(self):
        super().__init__("pipeline_tuning_logger")
        self._altitude = float("nan")
        self._t0 = None
        # Each entry: (t_sec, n_lines, altitude_m, [angle_deg, ...])
        self._rows = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(DVLAltitude, "/nautilus/dvl/altitude", self._alt_cb, qos)
        self.create_subscription(LineSegment2DArray, "/irls_line/lines", self._lines_cb, qos)

        self.get_logger().info(
            f"Logging to {OUTPUT_CSV} — drive the pipeline, then Ctrl+C for summary"
        )

    # ------------------------------------------------------------------
    def _now(self):
        t = self.get_clock().now().nanoseconds / 1e9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _alt_cb(self, msg):
        self._altitude = msg.altitude

    def _lines_cb(self, msg):
        t = self._now()
        angles = []
        for line in msg.lines:
            dx = line.p1.x - line.p0.x
            dy = line.p1.y - line.p0.y
            angles.append(math.degrees(math.atan2(dy, dx)) % 180.0)
        self._rows.append((t, len(msg.lines), self._altitude, angles))

    # ------------------------------------------------------------------
    def write_csv_and_summary(self):
        if not self._rows:
            self.get_logger().warn("No data recorded — was the mission running?")
            return

        # --- write CSV -------------------------------------------------
        with open(OUTPUT_CSV, "w") as f:
            f.write("t_sec,n_lines,altitude_m,ang0_deg,ang1_deg\n")
            for t, n, alt, angs in self._rows:
                a0 = f"{angs[0]:.1f}" if len(angs) > 0 else ""
                a1 = f"{angs[1]:.1f}" if len(angs) > 1 else ""
                f.write(f"{t:.3f},{n},{alt:.4f},{a0},{a1}\n")

        # --- statistics ------------------------------------------------
        total = len(self._rows)
        with_lines = sum(1 for _, n, _, _ in self._rows if n > 0)
        with_two   = sum(1 for _, n, _, _ in self._rows if n >= 2)
        detect_pct = 100.0 * with_lines / total if total else 0.0

        # duplicate: 2 lines whose angle difference is < 15 deg
        dup_frames = 0
        for _, n, _, angs in self._rows:
            if n >= 2:
                d = abs(angs[0] - angs[1]) % 180.0
                if d > 90.0:
                    d = 180.0 - d
                if d < 15.0:
                    dup_frames += 1
        dup_pct = 100.0 * dup_frames / with_two if with_two > 0 else 0.0

        # altitude stats (ignore NaN / negative)
        alts = [a for _, _, a, _ in self._rows if not math.isnan(a) and a > 0]
        alt_mean = sum(alts) / len(alts) if alts else float("nan")
        alt_min  = min(alts) if alts else float("nan")
        alt_max  = max(alts) if alts else float("nan")

        # gaps: consecutive zero-line frames
        gaps_sec = []
        gap_start = None
        seen_detection = False
        for t, n, _, _ in self._rows:
            if n > 0:
                if gap_start is not None:
                    gaps_sec.append(t - gap_start)
                    gap_start = None
                seen_detection = True
            else:
                if seen_detection and gap_start is None:
                    gap_start = t

        max_gap = max(gaps_sec) if gaps_sec else 0.0
        step = UPDATE_INTERVAL_MS / 1000.0
        recommended_nm = max(4, math.ceil(max_gap / step * 1.5))

        duration = self._rows[-1][0] - self._rows[0][0] if total > 1 else 0.0

        def _gap_count(threshold):
            return sum(1 for g in gaps_sec if g > threshold)

        # --- format summary --------------------------------------------
        lines = [
            "=" * 50,
            "  PIPELINE TUNING SUMMARY",
            "=" * 50,
            f"Duration                     : {duration:.1f}s",
            f"Total RANSAC frames          : {total}",
            "",
            "ALTITUDE",
            f"  mean={alt_mean:.3f}m  min={alt_min:.3f}m  max={alt_max:.3f}m",
            f"  pipe_px at mean alt (fy~180): {0.2 * 180 / alt_mean:.0f}px"
            if not math.isnan(alt_mean) else "",
            "",
            "RANSAC DETECTION",
            f"  Frames with >=1 line       : {with_lines}/{total} = {detect_pct:.0f}%",
            f"  Frames with 2 lines        : {with_two} ({100*with_two/total:.0f}%)",
            f"  Duplicates (<15deg apart)  : {dup_frames}/{with_two} = {dup_pct:.0f}% of 2-line frames",
            "",
            "MEASUREMENT GAPS (zero-detection runs)",
            f"  Total gaps                 : {len(gaps_sec)}",
            f"  Max gap                    : {max_gap:.2f}s",
            f"  Gaps > 0.4s (2 misses)    : {_gap_count(0.4)}",
            f"  Gaps > 0.8s (4 misses)    : {_gap_count(0.8)}",
            f"  Gaps > 1.2s (6 misses)    : {_gap_count(1.2)}",
            f"  Gaps > 2.0s (10 misses)   : {_gap_count(2.0)}",
            "",
            "RECOMMENDED PARAMETER ADJUSTMENTS",
            f"  delete_n / delete_m        : {recommended_nm}   (max_gap * 1.5 / {step}s)",
            f"  probability_of_detection   : {detect_pct / 100:.2f}",
            f"  (minTurnAngle dup residual : {dup_pct:.0f}% — should be near 0 with fix)",
            "",
            f"Full data  : {OUTPUT_CSV}",
            f"This file  : {OUTPUT_SUMMARY}",
        ]

        summary = "\n".join(lines)
        print("\n" + summary + "\n")
        with open(OUTPUT_SUMMARY, "w") as f:
            f.write(summary + "\n")


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = TuningLogger()

    def shutdown(sig, frame):
        node.write_csv_and_summary()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP, shutdown)

    try:
        rclpy.spin(node)
    except Exception:
        shutdown(None, None)


if __name__ == "__main__":
    main()
