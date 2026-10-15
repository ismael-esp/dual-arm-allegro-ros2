#!/usr/bin/env python3
"""
Thumb Gravity-Droop Diagnostic (joint 12) — verification experiment.

Purpose
-------
Confirm the ROOT CAUSE of the Allegro thumb sag: the driver's gravity
compensation uses a gravity vector HARDCODED in the hand base frame
(AllegroHandController.cpp:62 -> KDL::Vector(0,0,-9.80665)) that is NEVER
updated with the arm's live wrist orientation. If that is the cause, the
tracking error on the thumb base joint (index 12 / "j30") must GROW as the UR
wrist tilts the hand away from the orientation where hand-base +Z points up.

What this node does
-------------------
1. Holds ONE fixed 16-DOF hand pose by streaming it to
   /<ns>/setDesiredConfiguration at a fixed rate (it becomes the sole
   commander for that hand during the test).
2. Reads the ACTUAL hand joint state from /<ns>/joint_states.
3. Looks up TF <reference_frame> -> <hand_frame> and measures how far the
   hand frame's +Z axis tilts from world-up. That tilt equals the driver's
   gravity-model error.
4. Logs, per sample: wrist tilt (deg), commanded vs actual thumb angle, and
   the thumb tracking error. Writes a CSV and prints a live line + a final
   correlation summary (|thumb error| vs tilt).

IMPORTANT: You (the operator) move ONLY the UR wrist by hand (freedrive) or
teach pendant through several orientations while this runs. This node does NOT
move the arm.

Run:  ros2 run my_dual_robot_cell_control thumb_gravity_test --ros-args \
        -p hand_ns:=ra_ -p hand_frame:=ra_tool0 -p csv_path:=/tmp/thumb_test.csv
"""

import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

import tf2_ros

try:
    from allegro_hand_interfaces.srv import SetDesPos
    HAVE_SETDESPOS = True
except Exception:  # pragma: no cover - only if interfaces not sourced
    HAVE_SETDESPOS = False

N_DOF = 16
THUMB_IDX = 12  # joint_12_0 == "j30" == thumb base / abduction-roll joint
# Full thumb chain: 12=abduction/roll, 13=MCP, 14=PIP, 15=DIP flexion.
THUMB_JOINTS = [12, 13, 14, 15]


def quat_to_z_axis_in_world(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return the frame's +Z basis vector expressed in the parent (world) frame.

    This is the third column of the rotation matrix built from (x,y,z,w).
    """
    # Third column of R(quaternion): the image of local +Z in the parent frame.
    zx = 2.0 * (qx * qz + qw * qy)
    zy = 2.0 * (qy * qz - qw * qx)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    v = np.array([zx, zy, zz], dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    c = float(np.dot(a, b) / (na * nb))
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


class ThumbGravityTest(Node):
    def __init__(self):
        super().__init__("thumb_gravity_test")

        # ---- Parameters ----
        self.hand_ns = self.declare_parameter("hand_ns", "ra_").value
        # The hand base link. Use ra_palm_link / la_palm_link: it IS the frame
        # in which the driver's KDL solver assumes gravity == (0,0,-9.80665), so
        # its +Z tilt from world-up is exactly the gravity-model error. Only the
        # ORIENTATION of this frame matters (gravity is a free vector), so the
        # fact that palm_link's origin sits up near the fingers is irrelevant.
        # NOTE: do NOT use ra_tool0 here -- the flange->tool0 fixed joint adds a
        # 90 deg roll (rpy="pi/2 0 0"), so tool0's Z is 90 deg off from palm_link.
        self.hand_frame = self.declare_parameter("hand_frame", "ra_palm_link").value
        self.reference_frame = self.declare_parameter("reference_frame", "world").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)
        self.log_hz = float(self.declare_parameter("log_hz", 5.0).value)
        self.csv_path = self.declare_parameter("csv_path", "").value
        # Hold-pose selection:
        #   capture_current=True  -> snapshot the current 16-DOF pose at startup
        #                            (place the arm in the "thumb-up" nominal
        #                             orientation first). This is the default.
        #   capture_current=False -> use the explicit 16-value target_pose (rad).
        # NOTE: rclpy cannot infer the type of an empty-list default, so
        # target_pose defaults to a typed 16-zero DOUBLE_ARRAY.
        self.capture_current = bool(self.declare_parameter("capture_current", True).value)
        tp = self.declare_parameter("target_pose", [0.0] * N_DOF).value
        if self.capture_current:
            self.target_pose: Optional[List[float]] = None
        else:
            if len(tp) != N_DOF:
                raise RuntimeError(f"target_pose must have {N_DOF} values, got {len(tp)}")
            self.target_pose = [float(x) for x in tp]

        # The hand runs under ROS namespace "ra_" (or "la_"), which yields
        # /ra_/setDesiredConfiguration and /ra_/joint_states. Normalize the
        # user-supplied hand_ns (strip stray slashes) and rebuild with the
        # required slash after the namespace.
        ns = self.hand_ns.strip("/")
        self.srv_name = f"/{ns}/setDesiredConfiguration"
        self.state_topic = f"/{ns}/joint_states"

        if not HAVE_SETDESPOS:
            self.get_logger().error(
                "allegro_hand_interfaces not found. Source the workspace that "
                "builds allegro_hand_interfaces before running."
            )
            raise RuntimeError("allegro_hand_interfaces/SetDesPos unavailable")

        # ---- State ----
        self.latest_state: Optional[JointState] = None
        self.captured = self.target_pose is not None
        self.samples = []  # list of (tilt_world_deg, abs_thumb_err_deg)
        self._csv = None
        self._last_log = 0.0
        self._start_wall = time.time()
        self._z_start: Optional[np.ndarray] = None  # hand +Z in world at startup

        qos = QoSProfile(depth=10)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Subscriber (actual hand state) ----
        self.sub = self.create_subscription(JointState, self.state_topic, self._on_state, qos)

        # ---- Service client (command hold pose) ----
        self.cli = self.create_client(SetDesPos, self.srv_name)
        self.get_logger().info(f"Waiting for service {self.srv_name} ...")
        while rclpy.ok() and not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"  still waiting for {self.srv_name} (is the hand driver up?)")

        if self.csv_path:
            self._open_csv()

        # Command + log timers
        self.cmd_timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_cmd)
        self.log_timer = self.create_timer(1.0 / max(0.5, self.log_hz), self._on_log)

        self.get_logger().info(
            "ThumbGravityTest ready.\n"
            f"  hand_ns={ns}  service={self.srv_name}  state={self.state_topic}\n"
            f"  hand_frame={self.hand_frame}  reference_frame={self.reference_frame}\n"
            f"  thumb index={THUMB_IDX} (joint_12_0 / j30)\n"
            f"  target_pose={'explicit' if self.captured else 'CAPTURE current at startup'}\n"
            "ACTION: place the wrist so the hand-base points 'up' (nominal), then\n"
            "        slowly tilt ONLY the UR wrist through several orientations."
        )

    # ---------- helpers ----------
    def _open_csv(self):
        try:
            self._csv = open(self.csv_path, "w")
            self._csv.write(
                "t_rel,tilt_world_deg,tilt_from_start_deg,"
                "err12_deg,err13_deg,err14_deg,err15_deg,"
                "max_thumb_err_deg,mean_abs_err_deg\n"
            )
            self.get_logger().info(f"Logging CSV to {self.csv_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not open CSV {self.csv_path}: {e}")
            self._csv = None

    @staticmethod
    def _ordered_16(msg: JointState) -> Optional[List[float]]:
        """Extract joint_0_0..joint_15_0 in index order from a JointState.

        Matches by name suffix 'joint_<i>_0' (prefix-agnostic). Falls back to
        the raw position list if names don't match but there are >=16 values.
        """
        by_suffix = {}
        for name, pos in zip(msg.name, msg.position):
            for i in range(N_DOF):
                if name.endswith(f"joint_{i}_0"):
                    by_suffix[i] = float(pos)
                    break
        if len(by_suffix) == N_DOF:
            return [by_suffix[i] for i in range(N_DOF)]
        if len(msg.position) >= N_DOF:
            return [float(msg.position[i]) for i in range(N_DOF)]
        return None

    def _on_state(self, msg: JointState):
        self.latest_state = msg
        if not self.captured:
            ordered = self._ordered_16(msg)
            if ordered is not None:
                self.target_pose = ordered
                self.captured = True
                self.get_logger().info(
                    "Captured hold pose (rad): thumb[12]=%.4f | full=%s"
                    % (ordered[THUMB_IDX], ["%.3f" % v for v in ordered])
                )

    def _hand_tilt(self):
        """Return (tilt_from_world_up_deg, tilt_from_start_deg, z_in_world)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.hand_frame, rclpy.time.Time()
            )
        except Exception:
            return None, None, None
        q = tf.transform.rotation
        z = quat_to_z_axis_in_world(q.x, q.y, q.z, q.w)
        tilt_world = angle_between_deg(z, np.array([0.0, 0.0, 1.0]))
        if self._z_start is None:
            self._z_start = z
        tilt_start = angle_between_deg(z, self._z_start)
        return tilt_world, tilt_start, z

    # ---------- timers ----------
    def _on_cmd(self):
        if self.target_pose is None:
            return  # waiting for capture
        req = SetDesPos.Request()
        req.pos = list(self.target_pose)
        self.cli.call_async(req)

    def _on_log(self):
        if self.target_pose is None or self.latest_state is None:
            return
        cur = self._ordered_16(self.latest_state)
        if cur is None:
            return
        tilt_world, tilt_start, _ = self._hand_tilt()

        # Per-joint error for the whole thumb chain (12..15).
        errs = {j: math.degrees(self.target_pose[j] - cur[j]) for j in THUMB_JOINTS}
        max_thumb_err = max(errs.values(), key=abs)
        mean_abs_err_deg = math.degrees(
            float(np.mean(np.abs(np.array(self.target_pose) - np.array(cur))))
        )
        t_rel = time.time() - self._start_wall

        if tilt_world is not None:
            self.samples.append((tilt_world, abs(max_thumb_err)))

        tw = f"{tilt_world:6.1f}" if tilt_world is not None else "   n/a"
        ts = f"{tilt_start:6.1f}" if tilt_start is not None else "   n/a"
        self.get_logger().info(
            f"tilt(world)={tw}deg tilt(start)={ts}deg | thumb err[deg] "
            f"j12={errs[12]:+6.2f} j13={errs[13]:+6.2f} j14={errs[14]:+6.2f} "
            f"j15={errs[15]:+6.2f} | max|thumb|={abs(max_thumb_err):5.2f} "
            f"mean|all|={mean_abs_err_deg:5.2f}"
        )

        if self._csv is not None:
            tw_c = "" if tilt_world is None else f"{tilt_world:.4f}"
            ts_c = "" if tilt_start is None else f"{tilt_start:.4f}"
            self._csv.write(
                f"{t_rel:.3f},{tw_c},{ts_c},"
                f"{errs[12]:.4f},{errs[13]:.4f},{errs[14]:.4f},{errs[15]:.4f},"
                f"{max_thumb_err:.4f},{mean_abs_err_deg:.4f}\n"
            )
            self._csv.flush()

    # ---------- shutdown ----------
    def summarize(self):
        if len(self.samples) < 3:
            self.get_logger().info("Not enough samples for a correlation summary.")
            return
        arr = np.array(self.samples)
        tilt, err = arr[:, 0], arr[:, 1]
        if np.std(tilt) < 1e-6 or np.std(err) < 1e-6:
            corr = float("nan")
        else:
            corr = float(np.corrcoef(tilt, err)[0, 1])
        # Compare low-tilt vs high-tilt error buckets.
        med = float(np.median(tilt))
        lo = err[tilt <= med]
        hi = err[tilt > med]
        self.get_logger().info(
            "\n==================== SUMMARY ====================\n"
            f" samples                : {len(self.samples)}\n"
            f" tilt range (deg)          : {tilt.min():.1f} .. {tilt.max():.1f}\n"
            f" max|thumb err| range (deg): {err.min():.2f} .. {err.max():.2f}\n"
            f" corr(tilt, max|thumb err|): {corr:+.3f}\n"
            f" mean max|err| low tilt    : {lo.mean():.2f} deg (tilt<= {med:.1f})\n"
            f" mean max|err| high tilt   : {hi.mean():.2f} deg (tilt>  {med:.1f})\n"
            "=================================================\n"
            " Interpretation: a positive corr confirms the static-gravity-vector\n"
            " MECHANISM. But if the error magnitude is only ~1-3 deg, that\n"
            " mechanism is a MINOR contributor -- a table-hitting sag needs a\n"
            " much larger error, which points to an external/payload load hitting\n"
            " the 0.7 Nm torque cap or a higher-moment thumb pose. Reproduce the\n"
            " REAL failing task pose (and object, if any) to see the big error."
        )

    def destroy_node(self):
        try:
            self.summarize()
        finally:
            if self._csv is not None:
                try:
                    self._csv.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThumbGravityTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
