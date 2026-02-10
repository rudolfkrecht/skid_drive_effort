#!/usr/bin/env python3
"""
drive_effort_proxy_node.py

ROS 2 node that publishes a left/right *drive effort proxy* for skid-steer /
differential-drive robots.

It estimates an effective drive effort signal by inverting a simple,
control-oriented body dynamics model using:
  - wheel angular velocities from sensor_msgs/JointState.velocity
  - yaw rate from sensor_msgs/Imu.angular_velocity.z

Output:
  /drive_effort_lr (std_msgs/Float32MultiArray): [left_effort, right_effort]

Notes:
- This is NOT guaranteed to be physical motor torque (Nm). It is a model-consistent
  "effective effort" proxy. If you later replace the inputs with true torque telemetry,
  you can publish Nm on the same interface.
- Designed to work well on platforms (e.g., Clearpath Jackal) where JointState.effort is NaN.

Author: Rudolf Krecht (package scaffold)
License: MIT
"""

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class DriveEffortProxy(Node):
    def __init__(self) -> None:
        super().__init__("drive_effort_proxy")

        # ---- Topics ----
        self.declare_parameter("joint_states_topic", "/j100_0000/platform/joint_states")
        self.declare_parameter("imu_topic", "/j100_0000/sensors/imu_0/data")
        self.declare_parameter("out_topic", "/drive_effort_lr")

        # ---- Robot geometry ----
        self.declare_parameter("wheel_radius", 0.098)  # [m] (set per robot)

        # ---- Joint names ----
        self.declare_parameter("joint_fl", "front_left_wheel_joint")
        self.declare_parameter("joint_rl", "rear_left_wheel_joint")
        self.declare_parameter("joint_fr", "front_right_wheel_joint")
        self.declare_parameter("joint_rr", "rear_right_wheel_joint")

        # ---- Normalized body dynamics parameters (proxy model) ----
        # Longitudinal:  dv = b_v*tauS - f_c*tanh(v/v_eps) - a_v*v
        self.declare_parameter("a_v", 1.0)   # [1/s]
        self.declare_parameter("b_v", 0.08)  # [ (m/s^2) / effort ]
        self.declare_parameter("f_c", 0.20)  # [m/s^2]
        self.declare_parameter("v_eps", 0.03)  # [m/s] tanh smoothing

        # Yaw:          dr = b_r*tauD - a_r*r
        self.declare_parameter("a_r", 2.0)   # [1/s]
        self.declare_parameter("b_r", 0.10)  # [ (rad/s^2) / effort ]

        # ---- Filtering / safety ----
        self.declare_parameter("ema_alpha", 0.25)        # derivative smoothing (0..1)
        self.declare_parameter("max_abs_effort", 200.0)  # clamp

        # If you're bag-testing often, you can enable sim time in YAML:
        # self.declare_parameter("use_sim_time", True)

        # ---- Internal state ----
        self._omega: Dict[str, float] = {}  # joint_name -> rad/s
        self._r: Optional[float] = None     # yaw rate rad/s

        self._t_prev: Optional[float] = None
        self._v_prev: Optional[float] = None
        self._r_prev: Optional[float] = None

        self._dv_ema: float = 0.0
        self._dr_ema: float = 0.0

        # ---- ROS interfaces ----
        self._sub_js = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._cb_joint_states,
            50,
        )
        self._sub_imu = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._cb_imu,
            200,
        )
        self._pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("out_topic").value),
            10,
        )

        self.get_logger().info(
            "DriveEffortProxy started. Publishing effective effort on "
            f"{self.get_parameter('out_topic').value}"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def _cb_joint_states(self, msg: JointState) -> None:
        # Map velocities by joint name
        # NOTE: JointState.velocity might be empty if not provided.
        for name, vel in zip(msg.name, msg.velocity):
            try:
                self._omega[name] = float(vel)
            except Exception:
                continue

    def _cb_imu(self, msg: Imu) -> None:
        self._r = float(msg.angular_velocity.z)
        # Drive the estimation on IMU callback (typically higher-rate than joint_states)
        self._step_if_ready()

    # -------------------------
    # Core computations
    # -------------------------
    def _compute_v_from_wheels(self) -> Optional[float]:
        """
        v = (Rw/2) * (omega_L + omega_R), with omega_L/R as averages of front+rear.
        Wheel angular velocities are in rad/s.
        """
        fl = str(self.get_parameter("joint_fl").value)
        rl = str(self.get_parameter("joint_rl").value)
        fr = str(self.get_parameter("joint_fr").value)
        rr = str(self.get_parameter("joint_rr").value)

        if any(j not in self._omega for j in (fl, rl, fr, rr)):
            return None

        omega_L = 0.5 * (self._omega[fl] + self._omega[rl])
        omega_R = 0.5 * (self._omega[fr] + self._omega[rr])
        Rw = float(self.get_parameter("wheel_radius").value)

        return float(0.5 * Rw * (omega_L + omega_R))

    def _step_if_ready(self) -> None:
        if self._r is None:
            return

        v = self._compute_v_from_wheels()
        if v is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # Initialize history
        if self._t_prev is None or self._v_prev is None or self._r_prev is None:
            self._t_prev = now
            self._v_prev = v
            self._r_prev = self._r
            return

        dt = max(1e-3, now - self._t_prev)
        self._t_prev = now

        # Numerical derivatives
        dv = (v - self._v_prev) / dt
        dr = (self._r - self._r_prev) / dt
        self._v_prev = v
        self._r_prev = self._r

        # Smooth derivatives (EMA)
        alpha = float(self.get_parameter("ema_alpha").value)
        alpha = clip(alpha, 0.0, 1.0)
        self._dv_ema = (1.0 - alpha) * self._dv_ema + alpha * dv
        self._dr_ema = (1.0 - alpha) * self._dr_ema + alpha * dr

        # Load parameters
        a_v = float(self.get_parameter("a_v").value)
        b_v = float(self.get_parameter("b_v").value)
        f_c = float(self.get_parameter("f_c").value)
        v_eps = max(1e-3, float(self.get_parameter("v_eps").value))

        a_r = float(self.get_parameter("a_r").value)
        b_r = float(self.get_parameter("b_r").value)

        # Invert normalized dynamics to get required aggregated efforts
        # tauS ~ longitudinal aggregated effort, tauD ~ differential yaw effort
        tauS = (self._dv_ema + f_c * math.tanh(v / v_eps) + a_v * v) / max(b_v, 1e-6)
        tauD = (self._dr_ema + a_r * self._r) / max(b_r, 1e-6)

        # Convert to left/right "effort"
        eff_R = 0.5 * (tauS + tauD)
        eff_L = 0.5 * (tauS - tauD)

        # Clamp for sanity
        m = float(self.get_parameter("max_abs_effort").value)
        eff_L = clip(eff_L, -m, m)
        eff_R = clip(eff_R, -m, m)

        out = Float32MultiArray()
        out.data = [float(eff_L), float(eff_R)]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveEffortProxy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
