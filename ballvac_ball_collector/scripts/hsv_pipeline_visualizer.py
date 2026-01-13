#!/usr/bin/env python3
"""
HSV pipeline visualizer for the BallVac project.

Subscribe to a camera image topic, convert frames to HSV, apply per-color
thresholds + morphology, extract circular candidates, draw annotations and
show windows for debugging the pipeline.

Usage (ROS 2):
  ros2 run ballvac_ball_collector hsv_pipeline_visualizer.py --ros-args --params-file <params.yaml>
or run directly (with sourced workspace):
  python3 scripts/hsv_pipeline_visualizer.py --topic /camera/image_raw

Dependencies: rclpy, sensor_msgs, cv_bridge, numpy, opencv-python
"""
import argparse
import math
import sys

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_HSV_RANGES = {
    "red1": ((0, 100, 50), (10, 255, 255)),
    "red2": ((160, 100, 50), (179, 255, 255)),
    "green": ((40, 60, 50), (90, 255, 255)),
    "blue": ((100, 120, 50), (140, 255, 255)),
    "yellow": ((20, 100, 100), (35, 255, 255)),
    "orange": ((10, 100, 100), (20, 255, 255)),
    "purple": ((130, 50, 50), (160, 255, 255)),
    "cyan": ((80, 50, 50), (100, 255, 255)),
}


class HSVPipelineVisualizer(Node):
    def __init__(self, topic: str, hfov_deg: float = 60.0, ball_diameter_m: float = 0.066):
        super().__init__("hsv_pipeline_visualizer")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.img_cb, 1)
        self.hfov = math.radians(hfov_deg)
        self.ball_radius_m = ball_diameter_m / 2.0
        self.hsv_ranges = {k: (np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
                           for k, (lo, hi) in DEFAULT_HSV_RANGES.items()}
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.min_area = 200  # pixels
        self.get_logger().info(f"Subscribed to {topic}, hfov={hfov_deg} deg")

    def img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w = frame.shape[:2]
        focal_px = (w / 2.0) / math.tan(self.hfov / 2.0)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Combined mask for visualization
        combined_mask = np.zeros((h, w), dtype=np.uint8)

        detections = []

        for color_name, (lower, upper) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)
            # Morphological denoise
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

            # Accumulate for display
            combined_mask = cv2.bitwise_or(combined_mask, mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                if radius < 4:
                    continue

                # Distance estimate via similar triangles: z = f * r_real / r_pixels
                # r_real is ball radius in meters
                dist_m = (self.ball_radius_m * focal_px) / max(radius, 1.0)

                # Bearing angle relative to optical axis (radians)
                bearing = math.atan2((cx - (w / 2.0)), focal_px)

                # Confidence: normalized by area and radius heuristics
                conf = min(1.0, (area / (math.pi * (radius ** 2))) )

                detections.append({
                    "color": color_name,
                    "center": (int(cx), int(cy)),
                    "radius": int(radius),
                    "bearing_deg": math.degrees(bearing),
                    "distance_m": float(dist_m),
                    "confidence": float(conf),
                })

        # Draw detections on frame
        vis = frame.copy()
        for d in detections:
            cx, cy = d["center"]
            radius = d["radius"]
            color_bgr = (0, 255, 0)
            cv2.circle(vis, (cx, cy), radius, color_bgr, 2)
            label = f"{d['color']} {d['distance_m']:.2f}m {d['confidence']:.2f}"
            cv2.putText(vis, label, (cx - 30, cy - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        # Show windows
        cv2.imshow("original", vis)
        cv2.imshow("hsv", hsv)
        cv2.imshow("combined_mask", combined_mask)

        # Per-color mask windows (small)
        x_off = 0
        for color_name in list(self.hsv_ranges.keys()):
            mask = cv2.inRange(hsv, self.hsv_ranges[color_name][0], self.hsv_ranges[color_name][1])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
            small = cv2.resize(mask, (200, 150))
            win = f"mask_{color_name}"
            cv2.imshow(win, small)
            x_off += 200

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.get_logger().info("ESC pressed, shutting down windows")
            cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw", help="Image topic to subscribe")
    parser.add_argument("--hfov", type=float, default=60.0, help="Camera horizontal FOV in degrees")
    parser.add_argument("--ball_diameter", type=float, default=0.066, help="Ball diameter in meters")
    args, unknown = parser.parse_known_args(argv)

    rclpy.init(args=unknown)
    node = HSVPipelineVisualizer(topic=args.topic, hfov_deg=args.hfov, ball_diameter_m=args.ball_diameter)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
