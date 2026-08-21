#!/usr/bin/env python3
import os
import math
import csv
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # ZYX yaw-pitch-roll
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def ang_norm(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --- Parameters ---
        self.declare_parameter('file_name', 'waypoints.csv')
        self.declare_parameter('linear_speed', 0.20)          # m/s
        self.declare_parameter('angular_speed', 0.30)         # rad/s
        self.declare_parameter('yaw_threshold_deg', 3.0)      # deg
        self.declare_parameter('target_tolerance', 0.30)      # m

        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.yaw_threshold = math.radians(float(self.get_parameter('yaw_threshold_deg').value))
        self.target_tolerance = float(self.get_parameter('target_tolerance').value)

        path = os.path.expanduser(f'~/.ros/{file_name}')

        # --- Load waypoints (supports: UTM 7-col OR XY 2-col) ---
        self.waypoints: List[Tuple[float, float]] = []
        self.mode: str = "unknown"  # 'utm' or 'xy'
        self.ref_zone: Optional[str] = None
        try:
            self.waypoints, self.mode, self.ref_zone = self._load_waypoints_auto(path)
            if not self.waypoints:
                raise RuntimeError("No valid waypoints parsed.")
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoints (mode={self.mode}"
                + (f", utm_zone={self.ref_zone}" if self.ref_zone else "")
                + f") from {path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path}: {e}")
            raise

        # --- State ---
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.idx: int = 0
        self.completed: bool = False

        # --- IO ---
        self.pub = self.create_publisher(Twist, '/cmd_vel/follow', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/scout_mini_base_controller/odom', self._odom_cb, 10
        )
        self.sub_imu = self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.timer = self.create_timer(0.1, self._control_loop)  # 10 Hz

        self.get_logger().info(
            f"Params: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s, "
            f"yaw_thr={math.degrees(self.yaw_threshold):.1f} deg, tol={self.target_tolerance:.2f} m"
        )
        self.get_logger().info("Follower ready: using IMU yaw. No startup axis hacks.")

    # ---------------- CSV Loader ----------------
    def _load_waypoints_auto(self, path: str) -> Tuple[List[Tuple[float, float]], str, Optional[str]]:
        """
        - If CSV has >=7 columns and includes UTM info -> interpret as:
          [lat, lon, alt, status, utm_zone, easting, northing]  (header allowed)
          → Convert to local XY by subtracting the first (easting, northing).
        - Else if 2 columns -> treat as already-local XY.
        - Header line is auto-detected and skipped.
        """
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        with open(path, 'r', newline='') as f:
            reader = csv.reader(f)
            rows = [r for r in reader if len(r) > 0]

        if not rows:
            return [], "unknown", None

        def _is_number(s: str) -> bool:
            try:
                float(s)
                return True
            except Exception:
                return False

        header_present = False
        if not all(_is_number(x) for x in rows[0]):
            header_present = True
            header = [h.strip().lower() for h in rows[0]]
            data_rows = rows[1:]
        else:
            header = []
            data_rows = rows

        utm_idx_e = utm_idx_n = utm_idx_zone = None
        if header:
            name_map = {n: i for i, n in enumerate(header)}
            for cand in ['easting', 'utm_easting', 'x_utm', 'utm_x']:
                if cand in name_map:
                    utm_idx_e = name_map[cand]; break
            for cand in ['northing', 'utm_northing', 'y_utm', 'utm_y']:
                if cand in name_map:
                    utm_idx_n = name_map[cand]; break
            for cand in ['utm_zone', 'zone', 'utmzone']:
                if cand in name_map:
                    utm_idx_zone = name_map[cand]; break

        # Fallback positional assumption: [lat, lon, alt, status, utm_zone, easting, northing]
        if utm_idx_e is None and utm_idx_n is None and utm_idx_zone is None:
            if len(data_rows) > 0 and len(data_rows[0]) >= 7:
                utm_idx_zone = 4
                utm_idx_e = 5
                utm_idx_n = 6

        # Try UTM path
        if utm_idx_e is not None and utm_idx_n is not None:
            utm_pts: List[Tuple[float, float]] = []
            zones: List[str] = []
            for r in data_rows:
                try:
                    e = float(str(r[utm_idx_e]).strip())
                    n = float(str(r[utm_idx_n]).strip())
                    utm_pts.append((e, n))
                    z = str(r[utm_idx_zone]).strip() if utm_idx_zone is not None and len(r) > utm_idx_zone else ""
                    zones.append(z)
                except Exception:
                    continue

            if utm_pts:
                e0, n0 = utm_pts[0]
                local = [(e - e0, n - n0) for (e, n) in utm_pts]
                zone0 = zones[0] if zones else None
                if any(z and zone0 and z != zone0 for z in zones):
                    self.get_logger().warn("UTM zone mismatch in CSV; using first zone as reference.")
                return local, "utm", (zone0 if zone0 else None)

        # Else, 2-col XY
        xy: List[Tuple[float, float]] = []
        for r in data_rows:
            if len(r) < 2:
                continue
            try:
                x = float(str(r[0]).strip())
                y = float(str(r[1]).strip())
                xy.append((x, y))
            except Exception:
                continue

        if xy:
            return xy, "xy", None

        return [], "unknown", None

    # ---------------- Callbacks ----------------
    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    # ---------------- Control ----------------
    def _control_loop(self):
        if self.completed:
            return
        if self.current_x is None or self.current_y is None:
            return
        if self.current_yaw is None:
            return
        if not self.waypoints or self.idx >= len(self.waypoints):
            self._publish_stop()
            self.completed = True
            self.get_logger().info("No waypoints or already completed. Stopping.")
            return

        goal_x, goal_y = self.waypoints[self.idx]
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        dist = math.hypot(dx, dy)

        cmd = Twist()

        if dist <= self.target_tolerance:
            self.get_logger().info(f"Reached WP {self.idx+1}/{len(self.waypoints)} (dist={dist:.2f} m).")
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self._publish_stop()
                self.completed = True
                self.get_logger().info("All waypoints completed. Stopping.")
            return

        target_heading = math.atan2(dy, dx)
        yaw_err = ang_norm(target_heading - self.current_yaw)

        if abs(yaw_err) > self.yaw_threshold:
            cmd.angular.z = self.angular_speed if yaw_err > 0.0 else -self.angular_speed
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

    def _publish_stop(self):
        z = Twist()
        self.pub.publish(z)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
