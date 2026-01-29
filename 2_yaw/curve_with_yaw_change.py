import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from crazyflie_interfaces.msg import FullState

# pose 토픽 타입은 환경마다 PoseStamped / Pose일 수 있어서 둘 다 대비
try:
    from geometry_msgs.msg import PoseStamped as PoseMsg
    _POSE_IS_STAMPED = True
except Exception:
    from geometry_msgs.msg import Pose as PoseMsg
    _POSE_IS_STAMPED = False


# ----------------------------
# Math helpers
# ----------------------------
def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float):
    """roll=pitch=0 가정의 yaw-only quaternion (x,y,z,w)"""
    s = math.sin(yaw * 0.5)
    c = math.cos(yaw * 0.5)
    return (0.0, 0.0, s, c)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion -> yaw (rad)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def sat(x, lo, hi):
    return max(lo, min(hi, x))


# ----------------------------
# 7th polynomial (position/vel/acc boundary)
# c(t)=a0+a1 t+a2 t^2+...+a7 t^7
# ----------------------------
def solve_7th_poly(p0, v0, a0, p1, v1, a1, T):
    A = np.array([
        [T**4,    T**5,     T**6,     T**7],
        [4*T**3,  5*T**4,   6*T**5,   7*T**6],
        [12*T**2, 20*T**3,  30*T**4,  42*T**5],
        [24*T,    60*T**2,  120*T**3, 210*T**4]
    ], dtype=float)
    B = np.array([
        p1 - (p0 + v0*T + 0.5*a0*T**2),
        v1 - (v0 + a0*T),
        a1 - a0,
        0.0
    ], dtype=float)

    X = np.linalg.solve(A, B)
    return [float(p0), float(v0), float(a0/2.0), 0.0,
            float(X[0]), float(X[1]), float(X[2]), float(X[3])]


def get_state(c, t):
    pos = sum(coeff * (t**i) for i, coeff in enumerate(c))
    vel = sum(i * coeff * (t**(i-1)) for i, coeff in enumerate(c) if i > 0)
    acc = sum(i * (i-1) * coeff * (t**(i-2)) for i, coeff in enumerate(c) if i > 1)
    return float(pos), float(vel), float(acc)


# ----------------------------
# ROS2 Node
# ----------------------------
class LosFullState(Node):
    def __init__(self):
        super().__init__("los_cmd_full_state_yawrate")

        # Topics (cf1 기준)
        self.pose_topic = "/cf1/pose"
        self.cmd_topic = "/cf1/cmd_full_state"

        self.pub = self.create_publisher(FullState, self.cmd_topic, 10)

        # pose sub
        self.last_pose = None
        self.sub = self.create_subscription(PoseMsg, self.pose_topic, self._pose_cb, 10)

        self.get_logger().info(f"[TOPIC] pose_topic      = {self.pose_topic}")
        self.get_logger().info(f"[TOPIC] cmd_full_state = {self.cmd_topic}")
        self.get_logger().info("[SYSTEM] cmd_full_state로 S-curve + yaw(탄젠트) + yawrate 강제")

        # ----------------------------
        # Trajectory params (✅ 너가 요구한 “20cm 안 넘게” 반영)
        # ----------------------------
        self.T_SEG = 4.0

        # ✅ 높이 낮추기 (방 좁으면 0.25~0.35 권장)
        self.TARGET_Z = 0.5      # (m)

        # ✅ 전체 전진 거리 20cm 안쪽
        self.X_FINAL = 0.4      # (m) 18cm

        # ✅ 좌우 S 진폭 작게 (4cm면 총 폭 8cm)
        self.Y_LIMIT = 0.2      # (m)

        # ----------------------------
        # yaw control (✅ 방안 급회전 방지)
        # ----------------------------
        # 기존 200deg/s는 좁은 방에서 너무 세게 돎 → 60deg/s로 제한
        self.MAX_YAW_RATE = math.radians(20.0)   # rad/s
        self.K_YAW = 1.0                  # (기존 4.0 → 2.0) 부드럽게

        # ----------------------------
        # path waypoints (✅ 속도도 같이 줄여야 궤적이 안 벌어짐)
        # ----------------------------
        v_base = 0.02   # 5cm/s
        v_y = 0.015   # 3cm/s

        WPs = [
            {'p': [0.0,              0.0],         'v': [v_base, 0.0]},
            {'p': [self.X_FINAL/3.0,  self.Y_LIMIT],   'v': [v_base,  v_y]},
            {'p': [2.0*self.X_FINAL/3.0, -self.Y_LIMIT], 'v': [v_base, -v_y]},
            {'p': [self.X_FINAL,      0.0],         'v': [v_base, 0.0]}
        ]

        self.segments = []
        for i in range(len(WPs)-1):
            self.segments.append({
                'x': solve_7th_poly(WPs[i]['p'][0], WPs[i]['v'][0], 0.0,
                                    WPs[i+1]['p'][0], WPs[i+1]['v'][0], 0.0, self.T_SEG),
                'y': solve_7th_poly(WPs[i]['p'][1], WPs[i]['v'][1], 0.0,
                                    WPs[i+1]['p'][1], WPs[i+1]['v'][1], 0.0, self.T_SEG)
            })

        self.total_time = self.T_SEG * len(self.segments)

    def _pose_cb(self, msg):
        if _POSE_IS_STAMPED:
            self.last_pose = msg.pose
        else:
            self.last_pose = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _current_yaw(self) -> float:
        if self.last_pose is None:
            return 0.0
        q = self.last_pose.orientation
        return quat_to_yaw(q.x, q.y, q.z, q.w)

    def publish_full_state(self, x, y, z, vx, vy, vz, ax, ay, az, yaw_des, yaw_rate_cmd):
        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # position
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # orientation (yaw-only)
        qx, qy, qz, qw = yaw_to_quat(float(yaw_des))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # linear velocity
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        # angular velocity (여기가 yawrate)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(yaw_rate_cmd)   # rad/s

        # linear acceleration
        msg.acc.x = float(ax)
        msg.acc.y = float(ay)
        msg.acc.z = float(az)

        self.pub.publish(msg)

    def run(self):
        rate_hz = 100.0
        dt = 1.0 / rate_hz

        # ----------------------------
        # 0) ground hold
        # ----------------------------
        self.get_logger().info("--- [ARM/INIT] 0.5s hold at ground ---")
        t0 = self._now()
        while rclpy.ok() and (self._now() - t0) < 0.5:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_full_state(
                x=0.0, y=0.0, z=0.02,
                vx=0.0, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0,
                yaw_des=0.0,
                yaw_rate_cmd=0.0
            )
            time.sleep(dt)

        # ----------------------------
        # 1) takeoff ramp (cmd_full_state only)
        # ----------------------------
        self.get_logger().info("--- [TAKEOFF] cmdFullState ramp up ---")
        takeoff_T = 2.5
        t0 = self._now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            tau = self._now() - t0
            if tau >= takeoff_T:
                break
            z = 0.02 + (self.TARGET_Z - 0.02) * (tau / takeoff_T)
            self.publish_full_state(
                x=0.0, y=0.0, z=z,
                vx=0.0, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0,
                yaw_des=0.0,
                yaw_rate_cmd=0.0
            )
            time.sleep(dt)

        # ----------------------------
        # 2) flight
        # ----------------------------
        self.get_logger().info("--- [FLIGHT] S-curve + Tangent yaw + yawrate ---")
        start = self._now()
        last_print = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            t = self._now() - start
            if t >= self.total_time:
                break

            idx = min(int(t / self.T_SEG), len(self.segments)-1)
            t_rel = t - idx * self.T_SEG

            x, vx, ax = get_state(self.segments[idx]['x'], t_rel)
            y, vy, ay = get_state(self.segments[idx]['y'], t_rel)

            # 탄젠트 yaw
            speed_sq = vx*vx + vy*vy
            if speed_sq > 1e-4:
                yaw_des = math.atan2(vy, vx)
                # 곡률 기반 feedforward yawrate (rad/s)
                yaw_rate_ff = (vx * ay - vy * ax) / speed_sq
            else:
                yaw_des = 0.0
                yaw_rate_ff = 0.0

            # 현재 yaw(피드백)
            yaw_cur = self._current_yaw()
            yaw_err = normalize_angle(yaw_des - yaw_cur)

            # yawrate = FF + P
            yaw_rate_cmd = yaw_rate_ff + self.K_YAW * yaw_err
            yaw_rate_cmd = sat(yaw_rate_cmd, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)

            self.publish_full_state(
                x=x, y=y, z=self.TARGET_Z,
                vx=vx, vy=vy, vz=0.0,
                ax=ax, ay=ay, az=0.0,
                yaw_des=yaw_des,
                yaw_rate_cmd=yaw_rate_cmd
            )

            if t - last_print >= 0.5:
                last_print = t
                self.get_logger().info(
                    f"T:{t:4.1f} | PosSP:({x:+.2f},{y:+.2f}) "
                    f"| yaw_des={math.degrees(yaw_des):+.1f}deg "
                    f"| yaw_cur={math.degrees(yaw_cur):+.1f}deg "
                    f"| wz={math.degrees(yaw_rate_cmd):+.1f}deg/s"
                )

            time.sleep(dt)

        # ----------------------------
        # 3) land ramp
        # ----------------------------
        self.get_logger().info("--- [LAND] cmdFullState ramp down ---")
        land_T = 2.0
        t0 = self._now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            tau = self._now() - t0
            if tau >= land_T:
                break
            z = self.TARGET_Z * (1.0 - tau / land_T) + 0.02 * (tau / land_T)
            self.publish_full_state(
                x=self.X_FINAL, y=0.0, z=z,
                vx=0.0, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0,
                yaw_des=self._current_yaw(),
                yaw_rate_cmd=0.0
            )
            time.sleep(dt)

        self.get_logger().info("[DONE]")


def main():
    rclpy.init()
    node = LosFullState()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
